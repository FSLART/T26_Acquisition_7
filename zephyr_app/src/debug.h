/*
 * Debug snapshot (acq7) and periodic sensor log.
 *
 * acq7 is a global struct refreshed every 10 ms by the debug thread. Add "acq7" to a live view
 * (CLion Live Watches, STM32CubeIDE Live Expressions on zephyr.elf, STM32CubeMonitor) to watch the
 * whole board while it runs. Nothing in the firmware reads it.
 */

#ifndef T26_AQT7_DEBUG_H_
#define T26_AQT7_DEBUG_H_

#include <stdint.h>

#include "can_bus.h"
#include "can_tx.h"
#include "sensors.h"

struct acq7_debug {
    uint32_t time_ms;                         /* uptime (ms) */

    struct {
        uint16_t raw[SENSOR_ADC_CHANNELS];      /* latest samples: [0] PA7 brake, [1] PB0 susp L, [2] PB1 susp R, [3] PC4 NTC1 */
        uint16_t filtered[SENSOR_ADC_CHANNELS]; /* moving average */
        float volts[SENSOR_ADC_CHANNELS];       /* filtered value at the MCU pin (V) */
    } adc;

    struct {
        float brake_rear_bar;
        float brake_front_bar;                  /* 0 while 0x710 is not arriving */
        float susp_left_mm;
        float susp_right_mm;
        float ntc1_c;                           /* NTC_FAULT_C if open / shorted */
    } sensors;

    struct {
        uint8_t brake_light;                    /* 1 = light on (PC10 driven low) */
        uint8_t heartbeat;                      /* PC11 level */
    } io;

    struct {
        uint16_t coded;                         /* 0x710 FRT_BRK_PRESS, 0.1 bar/bit */
        uint8_t alive;                          /* 1 while 0x710 arrives within 500 ms */
        uint32_t age_ms;                        /* time since the last 0x710, UINT32_MAX if never */
        uint32_t rx_count;                      /* 0x710 frames received */
    } front;

    struct {
        struct can_tx_payloads tx;              /* last payloads queued on each bus */
        struct can_bus_status autonomous;       /* CAN1 */
        struct can_bus_status data;             /* CAN2 */
    } can;
};

/* The snapshot, at a fixed RAM address for the debugger */
extern struct acq7_debug acq7;

/* Starts the debug thread (snapshot every 10 ms, log every CONFIG_AQT7_DEBUG_PRINT_MS) */
void debug_start(void);

#endif /* T26_AQT7_DEBUG_H_ */
