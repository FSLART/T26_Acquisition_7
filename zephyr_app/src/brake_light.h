/*
 * Brake light output (PC10, active low).
 */

#ifndef T26_AQT7_BRAKE_LIGHT_H_
#define T26_AQT7_BRAKE_LIGHT_H_

/**
 * Configures PC10 as an output, light off.
 * @return 0 on success, negative errno otherwise
 */
int brake_light_init(void);

/* Starts the brake light thread. Call after brake_light_init() succeeded. */
void brake_light_start(void);

#endif /* T26_AQT7_BRAKE_LIGHT_H_ */
