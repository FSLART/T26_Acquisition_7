/*
 * IWDG supervisor.
 *
 * Each started thread checks in once per loop. main calls watchdog_service() every heartbeat:
 * the IWDG is fed only when every registered thread checked in since the last feed. A thread that
 * hangs (or a module that failed to start, see watchdog_fail) stops the feeding and the IWDG
 * resets the board after WATCHDOG_TIMEOUT_MS, like Error_Handler + IWDG in the CubeIDE firmware.
 */

#ifndef T26_AQT7_WATCHDOG_H_
#define T26_AQT7_WATCHDOG_H_

#include <zephyr/toolchain.h>

/* Same timeout as the CubeIDE firmware: LSI 32 kHz, prescaler 64, reload 1000 = 2 s */
#define WATCHDOG_TIMEOUT_MS 2000

/* Threads supervised by the watchdog (one bit each) */
enum watchdog_client {
    WDOG_SENSORS = 0,
    WDOG_BRAKE_LIGHT,
    WDOG_CAN_RX,
    WDOG_CAN_TX,
    WDOG_DEBUG,
    WDOG_CLIENT_COUNT
};

/**
 * Installs the timeout and starts the IWDG. Once started it cannot be stopped.
 * Call first in main, so a hang during the rest of the setup also resets the board.
 * @return 0 on success, negative errno otherwise
 */
int watchdog_init(void);

/* Adds a thread to the set that must check in. Call from the module's *_start(). */
void watchdog_register(enum watchdog_client client);

/* Called by a supervised thread once per loop iteration */
void watchdog_checkin(enum watchdog_client client);

/* Called by main every heartbeat: feeds the IWDG if every registered thread checked in */
void watchdog_service(void);

/**
 * Fatal error (a required module failed to start): stops feeding, so the board resets after
 * WATCHDOG_TIMEOUT_MS and the whole setup is tried again. Reboots directly if the IWDG isn't running.
 * Does not return.
 */
FUNC_NORETURN void watchdog_fail(const char *reason);

#endif /* T26_AQT7_WATCHDOG_H_ */
