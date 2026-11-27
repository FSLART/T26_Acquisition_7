/*
 * Unit tests for src/conversions.c: ADC counts -> bar, mm, C.
 *
 * The expected values are worked out by hand from the sensor data and the board circuit
 * (see the README, "Sensors"), not by calling the same formulas again. They depend on the
 * calibration in conversions.c (OFFSET_VOLTAGE = 0.614 V, CONVERSION_FACTOR = 1.0): after a
 * re-calibration, update the brake pressure values here too.
 *
 * Run (Linux or WSL, from the repository folder):
 *   west twister -T tests -p native_sim/native/64
 */

#include <zephyr/ztest.h>

#include "conversions.h"

#define ADC_FULL_SCALE 4095

/* ---------------------------------------------------------------------------------------------
 * Rear brake pressure: Bosch 0 261 545 053, 0.02857 V/bar, offset 0.614 V, ADC 3.3 V full scale
 * ------------------------------------------------------------------------------------------- */

ZTEST_SUITE(brake_pressure, NULL, NULL, NULL, NULL, NULL);

/* At or below the offset (pedal released) the pressure is exactly 0, never negative */
ZTEST(brake_pressure, test_zero_at_and_below_offset) {
    zassert_equal(MeasureBreakPressure(0), 0.0f);
    zassert_equal(MeasureBreakPressure(500), 0.0f);
    /* 761 counts = 0.6133 V, just under the 0.614 V offset */
    zassert_equal(MeasureBreakPressure(761), 0.0f);
    /* 762 counts = 0.6141 V, just over: a few thousandths of a bar */
    zassert_within(MeasureBreakPressure(762), 0.0f, 0.01f);
}

/* 2535 counts = 2.0429 V -> (2.0429 - 0.614) / 0.02857 = 50.01 bar */
ZTEST(brake_pressure, test_slope_at_50_bar) {
    zassert_within(MeasureBreakPressure(2535), 50.01f, 0.05f);
}

/* Highest reading possible: 3.3 V -> (3.3 - 0.614) / 0.02857 = 94.01 bar.
 * The 140 bar clamp can't be reached until a divider is fitted on PA7 (README, "Limitation").
 * When the divider is added this test fails on purpose: update it with the new range.
 */
ZTEST(brake_pressure, test_full_scale_is_94_bar) {
    zassert_within(MeasureBreakPressure(ADC_FULL_SCALE), 94.01f, 0.05f);
}

/* More counts never give less pressure (the brake light threshold depends on it) */
ZTEST(brake_pressure, test_monotonic) {
    float previous = MeasureBreakPressure(0);

    for (uint32_t counts = 1; counts <= ADC_FULL_SCALE; counts++) {
        float p = MeasureBreakPressure((uint16_t) counts);

        zassert_true(p >= previous, "pressure fell at %u counts", counts);
        zassert_true(p >= 0.0f && p <= 140.0f, "%.2f bar out of range at %u counts",
                     (double) p, counts);
        previous = p;
    }
}

/* 7 bar (brake light threshold) = 0.614 + 7 * 0.02857 = 0.814 V = 1010 counts */
ZTEST(brake_pressure, test_brake_light_threshold) {
    zassert_true(MeasureBreakPressure(1005) < 7.0f);
    zassert_true(MeasureBreakPressure(1015) >= 7.0f);
}

/* ---------------------------------------------------------------------------------------------
 * Suspension: 75 mm linear potentiometer, 5 V supply, 5 V -> 3.3 V divider
 * ------------------------------------------------------------------------------------------- */

ZTEST_SUITE(suspension, NULL, NULL, NULL, NULL, NULL);

ZTEST(suspension, test_end_points) {
    zassert_within(MeasureSuspensionPosition(0), 0.0f, 0.001f);
    /* 3.3 V at the pin = 5 V at the sensor = full 75 mm stroke */
    zassert_within(MeasureSuspensionPosition(ADC_FULL_SCALE), 75.0f, 0.01f);
}

/* 2048 counts = 2048 / 4095 of the stroke = 37.51 mm */
ZTEST(suspension, test_mid_stroke) {
    zassert_within(MeasureSuspensionPosition(2048), 37.51f, 0.01f);
}

/* Linear: every count adds the same 75 / 4095 = 0.0183 mm */
ZTEST(suspension, test_linear) {
    const float mm_per_count = 75.0f / ADC_FULL_SCALE;

    for (uint32_t counts = 0; counts <= ADC_FULL_SCALE; counts += 13) {
        zassert_within(MeasureSuspensionPosition((uint16_t) counts), counts * mm_per_count, 0.01f,
                       "not linear at %u counts", counts);
    }
}

/* ---------------------------------------------------------------------------------------------
 * Coolant NTC: Vishay NTCAIMM66H, 10 k @ 25 C, B25/85 = 3984 K.
 * 5 V -> NTC -> node A; node A: 100 k to GND, 24.3 k / 4.9 k divider to the ADC pin.
 * ------------------------------------------------------------------------------------------- */

ZTEST_SUITE(ntc, NULL, NULL, NULL, NULL, NULL);

/* 10 k at 25 C: node A = 5 V * 22.6 k / (10 k + 22.6 k) = 3.466 V -> pin 0.5817 V -> 722 counts */
ZTEST(ntc, test_25_c) {
    zassert_within(MeasureNTCTemperature(722), 25.0f, 0.1f);
}

/* 1066 ohm at 85 C (from B25/85) -> pin 0.8012 V -> 994 counts = 84.8 C */
ZTEST(ntc, test_85_c) {
    zassert_within(MeasureNTCTemperature(994), 84.8f, 0.3f);
}

/* 0 counts: sensor open (node A pulled to 0 V by the 100 k) */
ZTEST(ntc, test_open_sensor_is_fault) {
    zassert_equal(MeasureNTCTemperature(0), NTC_FAULT_C);
}

/* Node A at 5 V (NTC shorted): pin = 5 * 4.9 / 29.2 = 0.839 V = 1041.2 counts.
 * 1041 is still a (very hot) reading, 1042 and above are a fault.
 */
ZTEST(ntc, test_shorted_sensor_is_fault) {
    zassert_not_equal(MeasureNTCTemperature(1041), NTC_FAULT_C);
    zassert_true(MeasureNTCTemperature(1041) > 150.0f);

    for (uint32_t counts = 1042; counts <= ADC_FULL_SCALE; counts++) {
        zassert_equal(MeasureNTCTemperature((uint16_t) counts), NTC_FAULT_C,
                      "no fault at %u counts", counts);
    }
}

/* Over the valid range a higher voltage always means a hotter sensor */
ZTEST(ntc, test_monotonic) {
    float previous = MeasureNTCTemperature(1);

    for (uint32_t counts = 2; counts <= 1041; counts++) {
        float t = MeasureNTCTemperature((uint16_t) counts);

        zassert_true(t > previous, "temperature did not rise at %u counts", counts);
        previous = t;
    }
}
