/*
 * Sensor conversions: ADC counts -> engineering units.
 */

#include <math.h>

#include "conversions.h"

// Hardware constant definition
#define ADC_MAX 4095.0f
#define MCU_VOLTAGE 3.3f
#define SENSOR_VOLTAGE 5.0f

/**
 * Bosch 0 261 545 053: GAUGE pressure (0 bar = atmosphere, so 0 bar at rest), 0.5-4.5 V for 0-140 bar, 5 V supply.
 * The board divider does not attenuate: PA7 sees the sensor output ~1:1 (with 0.667 the rest read ~10 bar).
 * At rest (pedal released) PA7 reads ~0.61 V instead of the nominal 0.5 V (~4 bar too much). A divider can only
 * lower a voltage, so this is not the divider: it is the sensor's own offset, a sensor/board ground shift or residual
 * line pressure. OFFSET_VOLTAGE removes it. It has drifted before (0.52 V on 03-10), re-check it after harness work.
 * ponytail: ADC tops out at 3.3 V = ~98 bar; above that PA7 also exceeds its 3.6 V max. Fix the divider, then set its ratio here.
 *
 * @param adcReading bits read from the ADC
 * @return The brake pressure in bar (0-140 bar)
 */
float MeasureBreakPressure(uint16_t adcReading) {
    const float CONVERSION_FACTOR = 1.0f; // V_PA7 / V_sensor, measure both at rest to tune
    const float OFFSET_VOLTAGE = 0.614f; // PA7 volts at 0 bar: pedal released, copy acq7.adc.volts[0] here to re-zero
    const float SENSITIVITY = 0.02857f; // V/bar
    const float MAX_PRESSURE = 140.0f;

    float voltReading = ((float) adcReading * MCU_VOLTAGE / ADC_MAX) / CONVERSION_FACTOR;

    if (voltReading <= OFFSET_VOLTAGE) {
        // From old firmware: if below 0, set to 0.
        // However, if the voltage is below a 0.5V offset (assuming OFFSET_VOLTAGE), the pressure is 0.
        return 0.0f;
    }

    if (voltReading > SENSOR_VOLTAGE) {
        //Maybe return 140.0f immediately
        voltReading = SENSOR_VOLTAGE;
    }

    float pressure = (voltReading - OFFSET_VOLTAGE) / SENSITIVITY;

    //Pressure clamp
    if (pressure > MAX_PRESSURE) {
        return MAX_PRESSURE;
    }
    return pressure;
}

/**
 * Linear potentiometer: 75 mm electrical stroke, 5 V supply, 5 V -> 3.3 V divider
 * @param adcReading bits read from the ADC
 * @return Ride height in mm
 */
float MeasureSuspensionPosition(uint16_t adcReading) {
    const float CONVERSION_FACTOR = MCU_VOLTAGE / SENSOR_VOLTAGE;
    const float ELECTRICAL_STROKE = 75.0f;

    float volts = (float) adcReading * MCU_VOLTAGE / ADC_MAX / CONVERSION_FACTOR;

    return (ELECTRICAL_STROKE * volts) / SENSOR_VOLTAGE;
}

/**
 * Coolant NTCs: Vishay NTCAIMM66H, 10k @ 25 C, B25/85 = 3984 K.
 * Board: 5 V -> NTC -> node A; node A has 100k to GND and 24.3k -> ADC pin (4.9k from the ADC pin to GND)
 * @param adcReading bits read from the ADC
 * @return temperature in C
 */
float MeasureNTCTemperature(uint16_t adcReading) {
    const float NTC_R_SERIES = 24300.0f;
    const float NTC_R_BOTTOM = 4900.0f;
    const float NTC_R_PULLDOWN = 100000.0f;

    // Voltage at the MCU pin, then back up through the 24.3k / 4.9k divider to node A
    float v_pin = (float) adcReading * MCU_VOLTAGE / ADC_MAX;
    float v_a = v_pin * (NTC_R_SERIES + NTC_R_BOTTOM) / NTC_R_BOTTOM;

    // Open sensor (pulled to 0 V) or shorted sensor (node A at the full supply)
    if (adcReading == 0 || v_a >= SENSOR_VOLTAGE) {
        // Fault state
        return NTC_FAULT_C;
    }

    // Node A sees the 100k pull-down in parallel with the 24.3k + 4.9k branch
    float r_branch = NTC_R_SERIES + NTC_R_BOTTOM;
    float r_low = (NTC_R_PULLDOWN * r_branch) / (NTC_R_PULLDOWN + r_branch);

    // v_a = VSUPPLY * r_low / (r_ntc + r_low)
    float r_ntc = r_low * (SENSOR_VOLTAGE - v_a) / v_a;

    // Beta equation around 25 C
    const float T25_K = 298.15f;
    // 1 / T25_K + logf(r_ntc / NTC_R25) / NTC_B
    float inv_t = 1.0f / T25_K + logf(r_ntc / 10000.0f) / 3984.0f;

    return 1.0f / inv_t - 273.15f;
}
