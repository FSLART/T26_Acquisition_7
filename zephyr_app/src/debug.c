/*
 * Debug snapshot (acq7) and periodic sensor log.
 * Port of Acq7_DebugUpdate() and execute_100ms_tasks() from the STM32CubeIDE firmware.
 * Only reads the other modules through their getters.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "app.h"
#include "can_rx.h"
#include "debug.h"
#include "watchdog.h"

LOG_MODULE_REGISTER(debug, LOG_LEVEL_INF);

#define DEBUG_UPDATE_PERIOD_MS 10 /* same rate as Acq7_DebugUpdate() */

/* Not static: the debugger finds it by name */
struct acq7_debug acq7;

/* Read only: the pins are driven by brake_light.c and main.c */
static const struct gpio_dt_spec brake_light_pin = GPIO_DT_SPEC_GET(DT_ALIAS(brake_light), gpios);
static const struct gpio_dt_spec heartbeat_pin = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

static void debug_update(void) {
    struct vehicle_sensor_data acq7_data = sensors_get();
    struct front_brake_status front_brake_data = get_front_brake_status();

    acq7.time_ms = k_uptime_get_32();

    for (int i = 0; i < SENSOR_ADC_CHANNELS; i++) {
        acq7.adc.raw[i] = acq7_data.raw[i];
        acq7.adc.filtered[i] = acq7_data.filtered[i];
        acq7.adc.volts[i] = acq7_data.filtered[i] * 3.3f / 4095.0f;
    }

    acq7.sensors.brake_rear_bar = acq7_data.rear_brake_bar;
    acq7.sensors.brake_front_bar = front_brake_data.bar;
    acq7.sensors.susp_left_mm = acq7_data.susp_left_mm;
    acq7.sensors.susp_right_mm = acq7_data.susp_right_mm;
    acq7.sensors.ntc1_c = acq7_data.ntc1_c;

    /* Logical levels: brake_light 1 = on (the pin is active low) */
    acq7.io.brake_light = (uint8_t) gpio_pin_get_dt(&brake_light_pin);
    acq7.io.heartbeat = (uint8_t) gpio_pin_get_dt(&heartbeat_pin);

    acq7.front.coded = front_brake_data.coded;
    acq7.front.alive = front_brake_data.alive;
    acq7.front.age_ms = front_brake_data.age_ms;
    acq7.front.rx_count = front_brake_data.rx_count;

    acq7.can.tx = can_tx_get_last_payloads();
    acq7.can.autonomous = can_bus_get_status(CAN_BUS_AUTONOMOUS);
    acq7.can.data = can_bus_get_status(CAN_BUS_DATA);
}

/* Same content as the CubeIDE printf lines, from the snapshot just taken */
static void debug_print(void) {
    LOG_INF("Rear brake %.2f bar (adc %u) | Front brake %.2f bar (coded %u, %s)",
            (double) acq7.sensors.brake_rear_bar, acq7.adc.filtered[ADC_BRAKE],
            (double) acq7.sensors.brake_front_bar, acq7.front.coded,
            acq7.front.alive ? "alive" : "not arriving");
    LOG_INF("Susp L %.2f mm (adc %u) | Susp R %.2f mm (adc %u) | NTC1 %.1f C (adc %u)",
            (double) acq7.sensors.susp_left_mm, acq7.adc.filtered[ADC_SUSP_LEFT],
            (double) acq7.sensors.susp_right_mm, acq7.adc.filtered[ADC_SUSP_RIGHT],
            (double) acq7.sensors.ntc1_c, acq7.adc.filtered[ADC_NTC1]);
}

/**
 * Debug Thread: refreshes acq7 every 10 ms and logs the sensor values every
 * CONFIG_AQT7_DEBUG_PRINT_MS (0 = never). Lowest priority: it only uses spare CPU time.
 */
static void debug_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int64_t next = k_uptime_get();
    int64_t next_print = next;

    while (1) {
        watchdog_checkin(WDOG_DEBUG);

        debug_update();

        if ((CONFIG_AQT7_DEBUG_PRINT_MS > 0) && (k_uptime_get() >= next_print)) {
            debug_print();
            next_print += CONFIG_AQT7_DEBUG_PRINT_MS;
        }

        next += DEBUG_UPDATE_PERIOD_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(debug_tid, STACK_SIZE_DEBUG, debug_thread, NULL, NULL, NULL,
                PRIO_DEBUG, 0, SYS_FOREVER_MS);

void debug_start(void) {
    watchdog_register(WDOG_DEBUG);
    k_thread_start(debug_tid);
}
