/*
 * ADC1 sampling: brake pressure, suspension and coolant NTC.
 */

#ifndef T26_AQT7_SENSORS_H_
#define T26_AQT7_SENSORS_H_

#include <stdint.h>

/* Index of each signal in the zephyr,user io-channels list of the devicetree */
enum sensor_channel {
    ADC_BRAKE = 0,
    ADC_SUSP_LEFT = 1,
    ADC_SUSP_RIGHT = 2,
    ADC_NTC1 = 3,
    SENSOR_ADC_CHANNELS
};

/* Latest converted sensor values, written by the sensors thread */
struct vehicle_sensor_data {
    float rear_brake_bar;
    float susp_left_mm;  /* relative to ride height: + extension, - compression, 0 until zeroed */
    float susp_right_mm;
    float ntc1_c;        /* NTC_FAULT_C for an open or shorted sensor */
    uint16_t raw[SENSOR_ADC_CHANNELS];      /* latest sample per channel, ADC counts (for debugging) */
    uint16_t filtered[SENSOR_ADC_CHANNELS]; /* moving average per channel, ADC counts (for debugging) */
};

/**
 * Configures ADC1 and every channel.
 * @return 0 on success, negative errno otherwise
 */
int sensors_init(void);

/* Starts the sampling thread. Call after sensors_init() succeeded. */
void sensors_start(void);

/**
 * @return A consistent copy of the latest sensor values
 */
struct vehicle_sensor_data sensors_get(void);

#endif /* T26_AQT7_SENSORS_H_ */
