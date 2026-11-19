/*
 * Application-wide settings shared by the modules.
 */

#ifndef T26_AQT7_APP_H_
#define T26_AQT7_APP_H_

/*
 * Thread priorities: lower number = runs first (all preemptible).
 * Every thread is defined with SYS_FOREVER_MS as start delay, so it is created at boot but only
 * runs once main calls its module's *_start(), after the hardware it needs is set up.
 */
#define PRIO_SENSORS     2
#define PRIO_BRAKE_LIGHT 3
#define PRIO_CAN_RX      4
#define PRIO_CAN_TX      5
#define PRIO_DEBUG       6  /* lowest: snapshot and log only use spare CPU time */

#define STACK_SIZE_DEFAULT 1024
#define STACK_SIZE_DEBUG   2048 /* float formatting in the log */

#endif /* T26_AQT7_APP_H_ */
