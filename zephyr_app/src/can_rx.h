/*
 * CAN receive: list of frames to read on both buses, decoding with the DBC code.
 */

#ifndef T26_AQT7_CAN_RX_H_
#define T26_AQT7_CAN_RX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_bus.h"

/* Frames to receive, passed to can_bus_init() */
extern const struct can_rx_id can_rx_ids[];
extern const size_t can_rx_id_count;

/* Starts the CAN RX thread. Call after can_bus_init() succeeded. */
void can_rx_start(void);

/**
 * @return Front brake pressure (CAN1 0x710) in bar, 0 if it never arrived or stopped arriving
 */
float get_front_brake_pressure(void);

/* Everything known about 0x710, for debugging */
struct front_brake_status {
    float bar;         /* same as get_front_brake_pressure() */
    uint16_t coded;    /* last FRT_BRK_PRESS received, 0.1 bar/bit (kept when 0x710 stops) */
    bool alive;        /* 0x710 arrived within the last 500 ms */
    uint32_t age_ms;   /* time since the last 0x710, UINT32_MAX if never received */
    uint32_t rx_count; /* 0x710 frames decoded since boot */
};

struct front_brake_status get_front_brake_status(void);

#endif /* T26_AQT7_CAN_RX_H_ */
