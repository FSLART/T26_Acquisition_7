/*
 * Both CAN controllers: start, RX filters, RX queue, non-blocking send, statistics.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "can_bus.h"

LOG_MODULE_REGISTER(can_bus, LOG_LEVEL_INF);

static const struct device *const can_dev[CAN_BUS_COUNT] = {
    [CAN_BUS_AUTONOMOUS] = DEVICE_DT_GET(DT_ALIAS(can_autonomous)),
    [CAN_BUS_DATA] = DEVICE_DT_GET(DT_ALIAS(can_data)),
};

static const char *const bus_names[CAN_BUS_COUNT] = {
    [CAN_BUS_AUTONOMOUS] = "Autonomous (CAN1)",
    [CAN_BUS_DATA] = "DATA (CAN2)",
};

/* Filled by the RX callback (ISR), emptied by can_bus_receive() in the RX thread */
#define CAN_RX_QUEUE_LEN 16
K_MSGQ_DEFINE(can_rx_q, sizeof(struct can_rx_msg), CAN_RX_QUEUE_LEN, 4);

struct can_bus_stats can_stats[CAN_BUS_COUNT];

const char *can_bus_name(enum can_bus bus) {
    return (bus < CAN_BUS_COUNT) ? bus_names[bus] : "?";
}

/* RX filter callback, runs in the CAN interrupt: only queue the frame, decode in the RX thread */
static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data) {
    ARG_UNUSED(dev);
    enum can_bus bus = (enum can_bus) (uintptr_t) user_data;
    struct can_rx_msg msg = {.bus = bus, .frame = *frame};

    if (k_msgq_put(&can_rx_q, &msg, K_NO_WAIT) == 0) {
        atomic_inc(&can_stats[bus].rx);
    } else {
        atomic_inc(&can_stats[bus].rx_dropped);
    }
}

/* TX done callback, runs in the CAN interrupt: error 0 = acknowledged */
static void can_tx_callback(const struct device *dev, int error, void *user_data) {
    ARG_UNUSED(dev);
    enum can_bus bus = (enum can_bus) (uintptr_t) user_data;

    if (error == 0) {
        atomic_inc(&can_stats[bus].tx_ok);
    } else {
        atomic_inc(&can_stats[bus].tx_error);
    }
}

/* Meaning and usual causes (same texts as the CubeIDE firmware) */
static const char *const can_state_name[] = {
    [CAN_STATE_ERROR_ACTIVE] = "OK - error active, bus healthy",
    [CAN_STATE_ERROR_WARNING] = "WARNING - TEC or REC >= 96: errors happening, still communicating",
    [CAN_STATE_ERROR_PASSIVE] = "PASSIVE - TEC or REC >= 128: many errors (REC high = receive side, TEC high = our TX side)",
    [CAN_STATE_BUS_OFF] = "BUS-OFF - TEC > 255: not sending, hardware recovers after 128 x 11 recessive bits",
    [CAN_STATE_STOPPED] = "STOPPED - controller off (start failed: bus stuck dominant, transceiver unpowered), restarting",
};

static const char *const can_error_name[CAN_ERROR_TYPE_COUNT] = {
    [CAN_ERROR_NONE] = "none",
    [CAN_ERROR_STUFF] = "STUFF - 6 equal bits: node at another bitrate, noise, missing 120R termination",
    [CAN_ERROR_FORM] = "FORM - fixed bit field wrong: bitrate or sample point mismatch, noise",
    [CAN_ERROR_ACK] = "ACK - nobody acknowledged our frame: no other node powered/connected, CANH/CANL open or swapped, other bitrate",
    [CAN_ERROR_BIT_RECESSIVE] = "BIT RECESSIVE - sent 1 read 0: bus held dominant or shorted, faulty node/transceiver",
    [CAN_ERROR_BIT_DOMINANT] = "BIT DOMINANT - sent 0 read 1: our TX not reaching the bus (transceiver off/standby, TX wiring)",
    [CAN_ERROR_CRC] = "CRC - corrupted frame: noise, bad termination or long stubs, bitrate mismatch",
    [CAN_ERROR_RX_OVERRUN] = "RX OVERRUN - receive FIFO full: frames arrive faster than they are read",
};

const char *can_bus_state_text(enum can_state state) {
    return (state < ARRAY_SIZE(can_state_name)) ? can_state_name[state] : "?";
}

const char *can_bus_error_text(enum can_error_type error) {
    return (error < CAN_ERROR_TYPE_COUNT) ? can_error_name[error] : "?";
}

/* One per bus; must not live on the stack (the driver keeps a pointer to it) */
static struct can_state_change_callback can_state_cb[CAN_BUS_COUNT];

static atomic_t bus_off_count[CAN_BUS_COUNT];

/* Runs in the CAN interrupt: only counts bus-off entries. Logging is done (rate limited) by
 * can_bus_service(). Bus-off is recovered automatically by the driver.
 */
static void can_state_callback(const struct device *dev, struct can_state_change_callback *cb,
                               enum can_state state, struct can_bus_err_cnt err_cnt) {
    ARG_UNUSED(dev);
    ARG_UNUSED(err_cnt);
    enum can_bus bus = (enum can_bus) (cb - can_state_cb); /* which entry of can_state_cb[] */

    if (state == CAN_STATE_BUS_OFF) {
        atomic_inc(&bus_off_count[bus]);
    }
}

int can_bus_init(const struct can_rx_id *rx_ids, size_t rx_id_count) {
    for (int bus = 0; bus < CAN_BUS_COUNT; bus++) {
        const struct device *dev = can_dev[bus];

        if (!device_is_ready(dev)) {
            LOG_ERR("%s not ready", can_bus_name(bus));
            return -ENODEV;
        }

        can_init_state_change_callback(&can_state_cb[bus], can_state_callback);
        int ret = can_add_state_change_callback(dev, &can_state_cb[bus]);
        if (ret < 0) {
            LOG_WRN("%s state callback not installed (%d)", can_bus_name(bus), ret);
        }

        ret = can_start(dev);
        if (ret < 0) {
            LOG_ERR("%s start failed (%d)", can_bus_name(bus), ret);
            return ret;
        }
    }

    for (size_t i = 0; i < rx_id_count; i++) {
        const struct can_filter filter = {
            .id = rx_ids[i].id,
            .mask = CAN_STD_ID_MASK, /* exact ID */
            .flags = 0,              /* standard ID, data frames */
        };
        enum can_bus bus = rx_ids[i].bus;

        int ret = can_add_rx_filter(can_dev[bus], can_rx_callback, (void *) (uintptr_t) bus, &filter);
        if (ret < 0) {
            LOG_ERR("%s filter 0x%03x failed (%d)", can_bus_name(bus), rx_ids[i].id, ret);
            return ret;
        }
    }

    return 0;
}

int can_send_frame(enum can_bus bus, uint32_t id, const uint8_t *data, uint8_t len) {
    struct can_frame frame = {
        .id = id,
        .dlc = can_bytes_to_dlc(len),
        .flags = 0,
    };
    memcpy(frame.data, data, len);

    int ret = can_send(can_dev[bus], &frame, K_NO_WAIT, can_tx_callback, (void *) (uintptr_t) bus);
    if (ret < 0) {
        /* -EAGAIN mailboxes full, -ENETUNREACH bus-off, -ENETDOWN stopped */
        atomic_inc(&can_stats[bus].tx_dropped);
    }
    return ret;
}

int can_bus_receive(struct can_rx_msg *msg, k_timeout_t timeout) {
    return k_msgq_get(&can_rx_q, msg, timeout);
}

/* ---------------------------------------------------------------------------------------------
 * Bus health (port of CAN_Service)
 * ------------------------------------------------------------------------------------------- */

#define CAN_RESTART_MS 100  /* min time between restarts of a stopped controller */
#define CAN_REPORT_MS 1000  /* log rate limit per bus */

static struct can_bus_status bus_status[CAN_BUS_COUNT];
K_MUTEX_DEFINE(bus_status_lock);

/* Driver counters reset on every can_start(): totals since boot = base + driver counter */
static uint32_t error_base[CAN_BUS_COUNT][CAN_ERROR_TYPE_COUNT];
static int64_t last_restart_ms[CAN_BUS_COUNT];
static int64_t last_report_ms[CAN_BUS_COUNT];
static enum can_state reported_state[CAN_BUS_COUNT];       /* zero = CAN_STATE_ERROR_ACTIVE */
static enum can_error_type reported_error[CAN_BUS_COUNT];  /* zero = CAN_ERROR_NONE */

/* Error counters kept by the driver (CONFIG_CAN_STATS), since the last can_start() */
static void read_driver_errors(const struct device *dev, uint32_t out[CAN_ERROR_TYPE_COUNT]) {
    memset(out, 0, sizeof(uint32_t) * CAN_ERROR_TYPE_COUNT);
#ifdef CONFIG_CAN_STATS
    out[CAN_ERROR_STUFF] = can_stats_get_stuff_errors(dev);
    out[CAN_ERROR_FORM] = can_stats_get_form_errors(dev);
    out[CAN_ERROR_ACK] = can_stats_get_ack_errors(dev);
    out[CAN_ERROR_BIT_RECESSIVE] = can_stats_get_bit1_errors(dev);
    out[CAN_ERROR_BIT_DOMINANT] = can_stats_get_bit0_errors(dev);
    out[CAN_ERROR_CRC] = can_stats_get_crc_errors(dev);
    out[CAN_ERROR_RX_OVERRUN] = can_stats_get_rx_overruns(dev);
#else
    ARG_UNUSED(dev);
#endif
}

void can_bus_service(void) {
    int64_t now = k_uptime_get();

    for (int bus = 0; bus < CAN_BUS_COUNT; bus++) {
        const struct device *dev = can_dev[bus];

        k_mutex_lock(&bus_status_lock, K_FOREVER);
        struct can_bus_status s = bus_status[bus];
        k_mutex_unlock(&bus_status_lock);

        enum can_state state;
        struct can_bus_err_cnt err_cnt;
        if (can_get_state(dev, &state, &err_cnt) < 0) {
            continue;
        }
        s.state = state;
        s.tec = err_cnt.tx_err_cnt;
        s.rec = err_cnt.rx_err_cnt;

        /* Error totals; the type that grew the most since the last call becomes last_error */
        uint32_t driver[CAN_ERROR_TYPE_COUNT];
        read_driver_errors(dev, driver);

        enum can_error_type newest = CAN_ERROR_NONE;
        uint32_t newest_inc = 0;
        for (int t = CAN_ERROR_NONE + 1; t < CAN_ERROR_TYPE_COUNT; t++) {
            uint32_t total = error_base[bus][t] + driver[t];
            uint32_t inc = total - s.errors[t];

            if (inc > newest_inc) {
                newest_inc = inc;
                newest = t;
            }
            s.errors[t] = total;
        }
        if (newest != CAN_ERROR_NONE) {
            s.last_error = newest;
            s.last_error_ms = (uint32_t) now;
        }

        s.bus_off_count = atomic_get(&bus_off_count[bus]);
        s.tx_ok = atomic_get(&can_stats[bus].tx_ok);
        s.tx_error = atomic_get(&can_stats[bus].tx_error);
        s.tx_dropped = atomic_get(&can_stats[bus].tx_dropped);
        s.rx = atomic_get(&can_stats[bus].rx);
        s.rx_dropped = atomic_get(&can_stats[bus].rx_dropped);

        /* Log a change of state or of the last error type, at most once per second */
        if (((s.state != reported_state[bus]) || (s.last_error != reported_error[bus])) &&
            ((now - last_report_ms[bus]) >= CAN_REPORT_MS)) {
            last_report_ms[bus] = now;
            reported_state[bus] = s.state;
            reported_error[bus] = s.last_error;

            if (s.state == CAN_STATE_ERROR_ACTIVE) {
                LOG_INF("%s %s | TEC=%u REC=%u | last error: %s", can_bus_name(bus),
                        can_bus_state_text(s.state), s.tec, s.rec, can_bus_error_text(s.last_error));
            } else {
                LOG_WRN("%s %s | TEC=%u REC=%u | last error: %s", can_bus_name(bus),
                        can_bus_state_text(s.state), s.tec, s.rec, can_bus_error_text(s.last_error));
            }
        }

        /* Controller not running: restart it in place (filters are kept). Bus-off is not
         * included: the driver recovers from it by itself.
         */
        if ((s.state == CAN_STATE_STOPPED) && ((now - last_restart_ms[bus]) >= CAN_RESTART_MS)) {
            last_restart_ms[bus] = now;
            s.restarts++;

            /* can_start() resets the driver counters: keep the totals as the new base */
            memcpy(error_base[bus], s.errors, sizeof(error_base[bus]));

            int ret = can_start(dev);
            if (ret < 0) {
                LOG_WRN("%s restart failed (%d)", can_bus_name(bus), ret);
            }
        }

        k_mutex_lock(&bus_status_lock, K_FOREVER);
        bus_status[bus] = s;
        k_mutex_unlock(&bus_status_lock);
    }
}

struct can_bus_status can_bus_get_status(enum can_bus bus) {
    struct can_bus_status copy = {0};

    if (bus < CAN_BUS_COUNT) {
        k_mutex_lock(&bus_status_lock, K_FOREVER);
        copy = bus_status[bus];
        k_mutex_unlock(&bus_status_lock);
    }
    return copy;
}
