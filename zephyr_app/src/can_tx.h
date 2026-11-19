/*
 * CAN transmit: periodic frames of both buses, encoded with the DBC code.
 */

#ifndef T26_AQT7_CAN_TX_H_
#define T26_AQT7_CAN_TX_H_

#include <stdint.h>

#include "autonomous_t26.h"
#include "data_t26.h"

/* Last payload queued for each frame (port of acq7.can.tx_770_autonomous / tx_aqt7_data) */
struct can_tx_payloads {
    uint8_t autonomous_aqt7[AUTONOMOUS_T26_AQT7_LENGTH]; /* CAN1 0x770: rear brake pressure */
    uint8_t data_aqt7[DATA_T26_AQT7_LENGTH];             /* CAN2 0x770: SUSP_L, SUSP_R, NTC_1 */
};

/* Starts the CAN TX thread. Call after can_bus_init() succeeded. */
void can_tx_start(void);

/* Copy of the last payloads, for debugging */
struct can_tx_payloads can_tx_get_last_payloads(void);

#endif /* T26_AQT7_CAN_TX_H_ */
