/*
 * T26 Acquisition board 7 (AQT7), Zephyr port.
 * Based on "original" C code
 *
 * main starts the IWDG, sets up the hardware in order, starts each module's thread, then runs
 * the heartbeat and feeds the watchdog while every thread is alive.
 *
 * Every module is required, like in the CubeIDE firmware: if one fails to set up, watchdog_fail()
 * stops feeding and the board resets after 2 s and tries the whole setup again
 * (the CubeIDE firmware did the same through Error_Handler + IWDG).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "brake_light.h"
#include "can_bus.h"
#include "can_rx.h"
#include "can_tx.h"
#include "debug.h"
#include "fw_version.h"
#include "sensors.h"
#include "watchdog.h"

LOG_MODULE_REGISTER(aqt7, LOG_LEVEL_INF);

#define HEARTBEAT_PERIOD_MS 100

static const struct gpio_dt_spec heartbeat = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

int main(void) {
    LOG_INF("T26 AQT7 fw %s starting on %s", fw_version_string(), CONFIG_BOARD);

    /* First, so a hang anywhere in the setup below also resets the board.
     * If it fails the board still runs, unprotected: that is better than not running at all.
     */
    if (watchdog_init() < 0) {
        LOG_ERR("IWDG not running: the board will NOT reset on a hang");
    }

    if (!gpio_is_ready_dt(&heartbeat) || gpio_pin_configure_dt(&heartbeat, GPIO_OUTPUT_INACTIVE) < 0) {
        watchdog_fail("heartbeat GPIO setup failed");
    }

    // ADC1: brake pressure, suspension, NTC
    if (sensors_init() < 0) {
        watchdog_fail("ADC setup failed");
    }

    // PC10 brake light
    if (brake_light_init() < 0) {
        watchdog_fail("brake light setup failed");
    }

    // Both CAN buses: start + RX filters
    if (can_bus_init(can_rx_ids, can_rx_id_count) < 0) {
        watchdog_fail("CAN setup failed");
    }

    // All hardware ready: start the threads (each registers with the watchdog)
    sensors_start();
    brake_light_start();
    can_rx_start();
    can_tx_start();
    debug_start();

    /* Absolute deadlines: the period doesn't drift with the time spent in the loop */
    int64_t next = k_uptime_get();

    while (1) {
        gpio_pin_toggle_dt(&heartbeat);
        watchdog_service();
        can_bus_service(); /* bus health, error log, restart of a stopped controller */

        next += HEARTBEAT_PERIOD_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }

    return 0;
}
