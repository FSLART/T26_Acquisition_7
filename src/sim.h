/*
 * Simulated ADC inputs, set from the shell ("sim" command).
 *
 * An overridden channel replaces the raw ADC sample before the moving average, so the
 * simulated value goes through the same filter, conversions, brake light and CAN TX as a real one.
 */

#ifndef T26_AQT7_SIM_H_
#define T26_AQT7_SIM_H_

#include <stdint.h>

#include "sensors.h"

#if defined(CONFIG_AQT7_SIM)
/* Replaces the overridden channels of a fresh ADC sample. Called by the sensors thread. */
void sim_apply(uint16_t raw[SENSOR_ADC_CHANNELS]);
#else
static inline void sim_apply(uint16_t raw[SENSOR_ADC_CHANNELS]) {
    (void) raw;
}
#endif

#endif /* T26_AQT7_SIM_H_ */
