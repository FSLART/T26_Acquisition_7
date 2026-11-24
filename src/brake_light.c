/*
 * Brake light: PC10, driven from the rear (ADC) and front (CAN1 0x710) brake pressures.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "app.h"
#include "watchdog.h"
#include "brake_light.h"
#include "can_rx.h"
#include "sensors.h"

LOG_MODULE_REGISTER(brake_light, LOG_LEVEL_INF);

#define BRAKE_LIGHT_PERIOD_MS 50 //cycle duration
#define PRESSURE_LEVEL_REAR 7
#define PRESSURE_LEVEL_FRONT 7

static const struct gpio_dt_spec brake_light = GPIO_DT_SPEC_GET(DT_ALIAS(brake_light), gpios);

int brake_light_init(void) {
    if (!gpio_is_ready_dt(&brake_light)) {
        LOG_ERR("Brake light GPIO not ready");
        return -ENODEV;
    }

    /* Inactive = light off. The pin is active low in the devicetree, so this drives PC10 HIGH.
     * (The CubeIDE firmware boots with PC10 LOW, i.e. light on until the first 50 ms task.)
     */
    return gpio_pin_configure_dt(&brake_light, GPIO_OUTPUT_INACTIVE);
}

/**
 * Brake Light Thread
 * @param p1 UNUSED
 * @param p2 UNUSED
 * @param p3 UNUSED
 */
static void brake_light_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        watchdog_checkin(WDOG_BRAKE_LIGHT);

        bool is_on = sensors_get().rear_brake_bar >= PRESSURE_LEVEL_REAR
                     || get_front_brake_pressure() >= PRESSURE_LEVEL_FRONT;

        gpio_pin_set_dt(&brake_light, is_on);

        k_sleep(K_MSEC(BRAKE_LIGHT_PERIOD_MS));
    }
}

K_THREAD_DEFINE(brakelight_tid, STACK_SIZE_DEFAULT, brake_light_thread, NULL, NULL, NULL,
                PRIO_BRAKE_LIGHT, 0, SYS_FOREVER_MS);

void brake_light_start(void) {
    watchdog_register(WDOG_BRAKE_LIGHT);
    k_thread_start(brakelight_tid);
}
