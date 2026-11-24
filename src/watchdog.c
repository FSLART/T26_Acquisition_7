/*
 * IWDG supervisor: feeds the watchdog only while every supervised thread is alive.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>

#include "watchdog.h"

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

/* How long a thread may miss its check-in before it is reported (the reset comes at 2 s) */
#define WATCHDOG_REPORT_MS 1000

static const char *const client_name[WDOG_CLIENT_COUNT] = {
    [WDOG_SENSORS] = "sensors",
    [WDOG_BRAKE_LIGHT] = "brake_light",
    [WDOG_CAN_RX] = "can_rx",
    [WDOG_CAN_TX] = "can_tx",
    [WDOG_DEBUG] = "debug",
};

static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel = -1; /* -1 = IWDG not running */

static atomic_t expected; /* bit per registered thread */
static atomic_t alive;    /* bit per thread that checked in since the last feed */
static int64_t last_feed_ms;
static bool reported;     /* missing threads already logged for this stall */

int watchdog_init(void) {
    if (!device_is_ready(wdt)) {
        LOG_ERR("IWDG not ready");
        return -ENODEV;
    }

    const struct wdt_timeout_cfg cfg = {
        .window.min = 0,
        .window.max = WATCHDOG_TIMEOUT_MS,
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    int channel = wdt_install_timeout(wdt, &cfg);
    if (channel < 0) {
        LOG_ERR("IWDG timeout install failed (%d)", channel);
        return channel;
    }

    /* Paused while the debugger halts the CPU, so breakpoints don't reset the board */
    int ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (ret < 0) {
        LOG_ERR("IWDG setup failed (%d)", ret);
        return ret;
    }

    wdt_channel = channel;
    last_feed_ms = k_uptime_get();
    LOG_INF("IWDG running, %d ms timeout", WATCHDOG_TIMEOUT_MS);
    return 0;
}

void watchdog_register(enum watchdog_client client) {
    atomic_set_bit(&alive, client); /* counts as alive until its first loop */
    atomic_set_bit(&expected, client);
}

void watchdog_checkin(enum watchdog_client client) {
    atomic_set_bit(&alive, client);
}

void watchdog_service(void) {
    if (wdt_channel < 0) {
        return;
    }

    // each mask holds one bit per thread
    atomic_val_t expected_mask = atomic_get(&expected);
    atomic_val_t elapsed_mask = atomic_get(&alive);

    if ((elapsed_mask & expected_mask) == expected_mask) {
        wdt_feed(wdt, wdt_channel);
        atomic_clear(&alive);
        last_feed_ms = k_uptime_get();
        reported = false;
        return;
    }

    /* Not fed: say once which threads are missing, before the reset */
    if (!reported && (k_uptime_get() - last_feed_ms) >= WATCHDOG_REPORT_MS) {
        reported = true;
        for (int i = 0; i < WDOG_CLIENT_COUNT; i++) {
            if ((expected_mask & BIT(i)) && !(elapsed_mask & BIT(i))) {
                LOG_ERR("Thread %s stopped checking in: reset in < %d ms",
                        client_name[i], WATCHDOG_TIMEOUT_MS - WATCHDOG_REPORT_MS);
            }
        }
    }
}

FUNC_NORETURN void watchdog_fail(const char *reason) {
    LOG_ERR("Fatal: %s. Resetting to retry the setup", reason);

    if (wdt_channel < 0) {
        /* No IWDG to do it: let the log go out, then reset directly */
        k_sleep(K_MSEC(WATCHDOG_TIMEOUT_MS));
        sys_reboot(SYS_REBOOT_COLD);
    }

    /* Stop feeding: the IWDG resets the board within WATCHDOG_TIMEOUT_MS */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
