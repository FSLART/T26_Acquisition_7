/*
 * Sensor conversions: ADC counts -> engineering units.
 * Plain C, no Zephyr calls, so it can be unit tested on the PC (native_sim).
 */

#ifndef T26_AQT7_CONVERSIONS_H_
#define T26_AQT7_CONVERSIONS_H_

#include <stdint.h>

/* MeasureNTCTemperature result for an open or shorted sensor */
#define NTC_FAULT_C (-99.0f)

/**
 * @param adcReading bits read from the ADC
 * @return The brake pressure in bar (0-140 bar)
 */
float MeasureBreakPressure(uint16_t adcReading);

/**
 * @param adcReading bits read from the ADC
 * @return Absolute sensor position in mm (0-75 mm)
 */
float MeasureSuspensionPosition(uint16_t adcReading);

/**
 * @param adcReading bits read from the ADC
 * @return Temperature in C, NTC_FAULT_C for an open or shorted sensor
 */
float MeasureNTCTemperature(uint16_t adcReading);

#endif /* T26_AQT7_CONVERSIONS_H_ */
