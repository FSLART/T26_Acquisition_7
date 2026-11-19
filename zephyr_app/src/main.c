/*
 * T26 Acquisition board 7 (AQT7), Zephyr port.
 *
 * Skeleton: heartbeat on PC11 every 100 ms (same as the CubeIDE firmware), brake light
 * output set up and kept off, log over USART1. Sensors, CAN and watchdog are ported next.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "data_t26.h"

LOG_MODULE_REGISTER(aqt7, LOG_LEVEL_INF);

#define HEARTBEAT_PERIOD_MS 100

static const struct gpio_dt_spec heartbeat = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec brake_light = GPIO_DT_SPEC_GET(DT_ALIAS(brake_light), gpios);

static int outputs_init(void)
{
	if (!gpio_is_ready_dt(&heartbeat) || !gpio_is_ready_dt(&brake_light)) {
		LOG_ERR("GPIO controller not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&heartbeat, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return ret;
	}

	/* Inactive = light off. The pin is active low in the devicetree, so this drives PC10 HIGH.
	 * (The CubeIDE firmware boots with PC10 LOW, i.e. light on until the first 50 ms task.)
	 */
	return gpio_pin_configure_dt(&brake_light, GPIO_OUTPUT_INACTIVE);
}

int main(void)
{
	LOG_INF("T26 AQT7 starting on %s", CONFIG_BOARD);

	/* Checks that the DBC code links: pack an empty AQT7 frame */
	struct data_t26_aqt7_t aqt7 = {0};
	uint8_t frame[DATA_T26_AQT7_LENGTH];

	LOG_INF("AQT7: ID 0x%03x, pack returned %d", DATA_T26_AQT7_FRAME_ID,
		data_t26_aqt7_pack(frame, &aqt7, sizeof(frame)));

	if (outputs_init() < 0) {
		LOG_ERR("Output setup failed");
		return 0;
	}

	/* Absolute deadlines: the period doesn't drift with the time spent in the loop */
	int64_t next = k_uptime_get();

	while (1) {
		gpio_pin_toggle_dt(&heartbeat);

		next += HEARTBEAT_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}

	return 0;
}
