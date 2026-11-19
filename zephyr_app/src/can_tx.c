/*
 * CAN transmit: every periodic frame of both buses, each on its own absolute deadline.
 * To send a new frame: add a can_send_xxx() encoder and a next_xxx deadline in can_tx_thread.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "autonomous_t26.h"
#include "data_t26.h"

#include "app.h"
#include "watchdog.h"
#include "can_bus.h"
#include "can_tx.h"
#include "conversions.h"
#include "sensors.h"

LOG_MODULE_REGISTER(can_tx, LOG_LEVEL_INF);

#define CAN_AUTONOMOUS_AQT7_PERIOD_MS 50 /* CAN1 0x770, same as the CubeIDE firmware */
#define CAN_DATA_AQT7_PERIOD_MS 125      /* CAN2 0x770, 8 Hz */

/* Last payload passed to can_send_frame() for each frame, for debugging */
static struct can_tx_payloads last_payloads;
static struct k_spinlock last_payloads_lock;

struct can_tx_payloads can_tx_get_last_payloads(void) {
    k_spinlock_key_t key = k_spin_lock(&last_payloads_lock);
    struct can_tx_payloads copy = last_payloads;
    k_spin_unlock(&last_payloads_lock, key);
    return copy;
}

/* Autonomous bus (CAN1) 0x770 AQT7: rear brake pressure */
static void can_send_autonomous_aqt7(const struct vehicle_sensor_data *d) {
    struct autonomous_t26_aqt7_t msg = {
        .rear_brk_press = autonomous_t26_aqt7_rear_brk_press_encode(d->rear_brake_bar),
    };
    uint8_t data[AUTONOMOUS_T26_AQT7_LENGTH];

    if (autonomous_t26_aqt7_pack(data, &msg, sizeof(data)) < 0) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&last_payloads_lock);
    memcpy(last_payloads.autonomous_aqt7, data, sizeof(data));
    k_spin_unlock(&last_payloads_lock, key);

    can_send_frame(CAN_BUS_AUTONOMOUS, AUTONOMOUS_T26_AQT7_FRAME_ID, data, sizeof(data));
}

/* DATA bus (CAN2) 0x770 AQT7: suspension and coolant temperature */
static void can_send_data_aqt7(const struct vehicle_sensor_data *d) {
    struct data_t26_aqt7_t msg = {
        .susp_l = data_t26_aqt7_susp_l_encode(d->susp_left_mm),
        .susp_r = data_t26_aqt7_susp_r_encode(d->susp_right_mm),
    };

    /* NTC_1 is uint8, 1 degC/bit: rounded and clamped to 0..254, 255 = open / shorted sensor */
    if (d->ntc1_c <= NTC_FAULT_C) {
        msg.ntc_1 = 255;
    } else {
        float t = CLAMP(d->ntc1_c, 0.0f, 254.0f);
        msg.ntc_1 = data_t26_aqt7_ntc_1_encode(t + 0.5f);
    }

    uint8_t data[DATA_T26_AQT7_LENGTH];

    if (data_t26_aqt7_pack(data, &msg, sizeof(data)) < 0) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&last_payloads_lock);
    memcpy(last_payloads.data_aqt7, data, sizeof(data));
    k_spin_unlock(&last_payloads_lock, key);

    can_send_frame(CAN_BUS_DATA, DATA_T26_AQT7_FRAME_ID, data, sizeof(data));
}

/**
 * CAN TX Thread: sends every periodic frame of both buses, each on its own absolute deadline.
 */
static void can_tx_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next_autonomous_aqt7 = k_uptime_get();
    int64_t next_data_aqt7 = next_autonomous_aqt7;

    while (1) {
        watchdog_checkin(WDOG_CAN_TX);

        int64_t now = k_uptime_get();
        struct vehicle_sensor_data d = sensors_get();

        if (now >= next_autonomous_aqt7) {
            can_send_autonomous_aqt7(&d);
            next_autonomous_aqt7 += CAN_AUTONOMOUS_AQT7_PERIOD_MS;
        }

        if (now >= next_data_aqt7) {
            can_send_data_aqt7(&d);
            next_data_aqt7 += CAN_DATA_AQT7_PERIOD_MS;
        }

        /* Add new periodic frames with their own next_* deadline and include it here */
        k_sleep(K_TIMEOUT_ABS_MS(MIN(next_autonomous_aqt7, next_data_aqt7)));
    }
}

K_THREAD_DEFINE(can_tx_tid, STACK_SIZE_DEFAULT, can_tx_thread, NULL, NULL, NULL,
                PRIO_CAN_TX, 0, SYS_FOREVER_MS);

void can_tx_start(void) {
    watchdog_register(WDOG_CAN_TX);
    k_thread_start(can_tx_tid);
}
