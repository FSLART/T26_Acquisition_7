/*
 * ADC1 sampling: one sample of every channel each SAMPLE_PERIOD_MS, moving average of AVG_SAMPLES,
 * conversion to engineering units, suspension zeroing.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#include "app.h"
#include "watchdog.h"
#include "conversions.h"
#include "sensors.h"

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

/* Window = 10 ms * 10 = 100 ms. (The CubeIDE firmware sampled every 50 ms, a ~500 ms window.) */
#define SAMPLE_PERIOD_MS 10
#define AVG_SAMPLES 10

/* DT_FOREACH_PROP_ELEM passes (node_id, prop, idx); ADC_DT_SPEC_GET_BY_IDX only takes (node_id, idx) */
#define ADC_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Index order from zephyr,user in the dts: 0 brake pressure, 1 susp left, 2 susp right, 3 NTC */
static const struct adc_dt_spec ch[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, ADC_SPEC_AND_COMMA)
};

BUILD_ASSERT(ARRAY_SIZE(ch) == SENSOR_ADC_CHANNELS,
             "zephyr,user io-channels and enum sensor_channel are out of sync");

/* The STM32 driver returns one sample per channel, in channel number order (7, 8, 9, 14).
 * That matches the ch[] order, so raw_adc_data[i] belongs to ch[i].
 */
static uint16_t raw_adc_data[SENSOR_ADC_CHANNELS];
static struct adc_sequence seq = {
    .buffer = raw_adc_data,
    .buffer_size = sizeof(raw_adc_data),
};

static struct vehicle_sensor_data vehicle_data;
K_MUTEX_DEFINE(status_lock);

struct vehicle_sensor_data sensors_get(void) {
    k_mutex_lock(&status_lock, K_FOREVER);
    struct vehicle_sensor_data copy = vehicle_data;
    k_mutex_unlock(&status_lock);
    return copy;
}

int sensors_init(void) {
    for (size_t i = 0; i < SENSOR_ADC_CHANNELS; i++) {
        if (!adc_is_ready_dt(&ch[i])) {
            LOG_ERR("ADC %s not ready", ch[i].dev->name);
            return -ENODEV;
        }

        int ret = adc_channel_setup_dt(&ch[i]);
        if (ret < 0) {
            LOG_ERR("ADC channel %d setup failed (%d)", ch[i].channel_id, ret);
            return ret;
        }

        seq.channels |= BIT(ch[i].channel_id);
    }

    /* Resolution and oversampling come from the devicetree (12 bit) */
    return adc_sequence_init_dt(&ch[0], &seq);
}

/**
 * Sensors Thread: samples every ADC channel each SAMPLE_PERIOD_MS, filters with a moving average,
 * converts to engineering units and publishes the result in vehicle_data.
 */
static void sensor_sampling_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    // Moving average: circular buffer + running sum, so each update is O(channels)
    static uint16_t history[SENSOR_ADC_CHANNELS][AVG_SAMPLES];
    uint32_t sum[SENSOR_ADC_CHANNELS] = {0};
    uint16_t filtered[SENSOR_ADC_CHANNELS];
    uint8_t pos = 0;
    uint8_t count = 0; /* samples in the buffer, up to AVG_SAMPLES */

    // Ride height: the first full average is taken as 0 mm
    bool was_suspension_zeroed = false;
    float left_suspension_zero = 0.0f;
    float right_suspension_zero = 0.0f;

    int64_t next = k_uptime_get();

    while (1) {
        watchdog_checkin(WDOG_SENSORS);

        int ret = adc_read_dt(&ch[0], &seq);

        if (ret < 0) {
            LOG_WRN("ADC read failed (%d)", ret);
        } else {
            for (size_t channel = 0; channel < SENSOR_ADC_CHANNELS; channel++) {
                sum[channel] += raw_adc_data[channel];
                sum[channel] -= history[channel][pos];
                history[channel][pos] = raw_adc_data[channel];
            }

            // Next buffer position
            pos = (pos + 1) % AVG_SAMPLES;
            if (count < AVG_SAMPLES) {
                count++;
            }

            //Complete avg calculation
            for (size_t channel = 0; channel < SENSOR_ADC_CHANNELS; channel++) {
                filtered[channel] = (uint16_t) (sum[channel] / count);
            }

            float left_suspension_abs = MeasureSuspensionPosition(filtered[ADC_SUSP_LEFT]);
            float right_suspension_abs = MeasureSuspensionPosition(filtered[ADC_SUSP_RIGHT]);

            if (!was_suspension_zeroed && count == AVG_SAMPLES) {
                left_suspension_zero = left_suspension_abs;
                right_suspension_zero = right_suspension_abs;
                was_suspension_zeroed = true;
                LOG_INF("Suspension zeroed: L %.1f mm, R %.1f mm",
                        (double) left_suspension_zero, (double) right_suspension_zero);
            }

            // Sign convention (sent as SUSP_L / SUSP_R on the DATA bus):
            //   positive (+) = EXTENSION   (wheel drops, suspension opens past ride height)
            //   negative (-) = COMPRESSION (wheel rises, suspension closes past ride height)
            // The sensor voltage goes DOWN when the suspension extends on this car, hence ZERO - abs.
            struct vehicle_sensor_data data_sample = {
                .rear_brake_bar = MeasureBreakPressure(filtered[ADC_BRAKE]),
                .susp_left_mm = was_suspension_zeroed ? left_suspension_zero - left_suspension_abs : 0.0f,
                .susp_right_mm = was_suspension_zeroed ? right_suspension_zero - right_suspension_abs : 0.0f,
                .ntc1_c = MeasureNTCTemperature(filtered[ADC_NTC1]),
            };
            memcpy(data_sample.raw, raw_adc_data, sizeof(data_sample.raw));
            memcpy(data_sample.filtered, filtered, sizeof(data_sample.filtered));

            k_mutex_lock(&status_lock, K_FOREVER);
            vehicle_data = data_sample;
            k_mutex_unlock(&status_lock);
        }

        next += SAMPLE_PERIOD_MS;
        k_sleep(K_TIMEOUT_ABS_MS(next));
    }
}

K_THREAD_DEFINE(sensors_tid, STACK_SIZE_DEFAULT, sensor_sampling_thread, NULL, NULL, NULL,
                PRIO_SENSORS, 0, SYS_FOREVER_MS);

void sensors_start(void) {
    watchdog_register(WDOG_SENSORS);
    k_thread_start(sensors_tid);
}
