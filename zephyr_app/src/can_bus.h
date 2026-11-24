/*
 * Both CAN controllers: start, RX filters, RX queue, non-blocking send, statistics.
 * Knows nothing about the DBC: encoding is in can_tx.c, decoding in can_rx.c.
 */

#ifndef T26_AQT7_CAN_BUS_H_
#define T26_AQT7_CAN_BUS_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

// The numbers here are not to be confused with any possible numeration given to the buses in other documentation
enum can_bus {
    CAN_BUS_AUTONOMOUS = 0, /* CAN1, PA11/PA12 (alias can-autonomous) */
    CAN_BUS_DATA,           /* CAN2, PB12/PB13 (alias can-data) */
    CAN_BUS_COUNT
};

/* A frame to receive: standard 11-bit ID, exact match. One hardware filter per entry. */
struct can_rx_id {
    enum can_bus bus;
    uint32_t id;
};

/* Received frame + the bus it came from */
struct can_rx_msg {
    enum can_bus bus;
    struct can_frame frame;
};

/* Per bus counters, for debugging (read them in the debugger or log them) */
struct can_bus_stats {
    atomic_t tx_ok;      /* frames acknowledged on the bus */
    atomic_t tx_error;   /* frames that failed after being queued (bus-off, stopped...) */
    atomic_t tx_dropped; /* not queued (mailboxes full, bus-off): the next period sends a fresh one */
    atomic_t rx;         /* frames received through the filters */
    atomic_t rx_dropped; /* RX queue full */
};

extern struct can_bus_stats can_stats[CAN_BUS_COUNT];

/* Protocol error types, as reported by the controller (bxCAN last error code) */
enum can_error_type {
    CAN_ERROR_NONE = 0,
    CAN_ERROR_STUFF,
    CAN_ERROR_FORM,
    CAN_ERROR_ACK,
    CAN_ERROR_BIT_RECESSIVE,
    CAN_ERROR_BIT_DOMINANT,
    CAN_ERROR_CRC,
    CAN_ERROR_RX_OVERRUN,
    CAN_ERROR_TYPE_COUNT
};

/* Health of one bus, refreshed by can_bus_service() (port of CAN_Service / CAN_BusStatus) */
struct can_bus_status {
    enum can_state state;
    uint8_t tec;                             /* transmit error counter */
    uint8_t rec;                             /* receive error counter */
    uint32_t errors[CAN_ERROR_TYPE_COUNT];   /* errors of each type since boot ([CAN_ERROR_NONE] unused) */
    enum can_error_type last_error;          /* most recent error type, CAN_ERROR_NONE if none yet */
    uint32_t last_error_ms;                  /* uptime of last_error */
    uint32_t bus_off_count;                  /* times the bus went bus-off */
    uint32_t restarts;                       /* controller restarts by can_bus_service() */
    uint32_t tx_ok, tx_error, tx_dropped, rx, rx_dropped; /* copies of can_stats[] */
};

/**
 * Call periodically (main does it every 100 ms). For each bus: reads the state, error counters
 * and error types; restarts a stopped controller (at most every 100 ms); logs a change of state
 * or of the last error type (at most once per second per bus).
 */
void can_bus_service(void);

/* Copy of the latest health of a bus, as refreshed by can_bus_service() */
struct can_bus_status can_bus_get_status(enum can_bus bus);

/* Readable texts for logs and the debugger */
const char *can_bus_state_text(enum can_state state);
const char *can_bus_error_text(enum can_error_type error);

/**
 * Starts both CAN controllers and installs one RX filter per entry of rx_ids.
 * Bitrate and sample point come from the devicetree. CAN_MODE_NORMAL retransmits until
 * acknowledged, like the CubeIDE firmware (AutoRetransmission = ENABLE).
 * @return 0 on success, negative errno otherwise
 */
int can_bus_init(const struct can_rx_id *rx_ids, size_t rx_id_count);

/**
 * Queues one standard-ID data frame. Never blocks: if every TX mailbox is busy the value is
 * dropped (counted) and the next period sends a fresh one, like the CubeIDE CAN_Send.
 * @return 0 if queued, negative errno otherwise
 */
int can_send_frame(enum can_bus bus, uint32_t id, const uint8_t *data, uint8_t len);

/**
 * Takes the next received frame (any bus) from the RX queue.
 * @return 0 on success, -EAGAIN on timeout
 */
int can_bus_receive(struct can_rx_msg *msg, k_timeout_t timeout);

/* Name for logs, e.g. "Autonomous (CAN1)" */
const char *can_bus_name(enum can_bus bus);

#endif /* T26_AQT7_CAN_BUS_H_ */
