/*
 * "sim" shell command: overrides ADC channels with values typed on the console.
 *
 *   sim set <channel> <counts>   raw ADC value, 0-4095
 *   sim mv <channel> <mV>        voltage at the MCU pin, 0-3300 mV
 *   sim off [channel]            back to the real input (all channels if none given)
 *   sim show                     override state and the resulting sensor values
 *
 * <channel> is brake, susp_l, susp_r, ntc or its index 0-3 (Tab completes the names).
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include "sensors.h"
#include "sim.h"

LOG_MODULE_REGISTER(sim, LOG_LEVEL_INF);

#define SIM_OFF  (-1)
#define ADC_MAX  4095
#define VREF_MV  3300

/* Raw value per channel, SIM_OFF = real input. Atomic: written by the shell, read by the sensors thread. */
static atomic_t override[SENSOR_ADC_CHANNELS] = {
    [0 ... SENSOR_ADC_CHANNELS - 1] = ATOMIC_INIT(SIM_OFF),
};

/* Same order as enum sensor_channel */
static const char *const channel_names[SENSOR_ADC_CHANNELS] = {
    [ADC_BRAKE] = "brake",
    [ADC_SUSP_LEFT] = "susp_l",
    [ADC_SUSP_RIGHT] = "susp_r",
    [ADC_NTC1] = "ntc",
};

void sim_apply(uint16_t raw[SENSOR_ADC_CHANNELS]) {
    for (size_t i = 0; i < SENSOR_ADC_CHANNELS; i++) {
        atomic_val_t v = atomic_get(&override[i]);
        if (v != SIM_OFF) {
            raw[i] = (uint16_t) v;
        }
    }
}

/* @return the channel index, or -1 (with an error printed) for an unknown name */
static int parse_channel(const struct shell *sh, const char *arg) {
    for (int i = 0; i < SENSOR_ADC_CHANNELS; i++) {
        if (strcmp(arg, channel_names[i]) == 0) {
            return i;
        }
    }

    int err = 0;
    long idx = shell_strtol(arg, 10, &err);
    if (err == 0 && idx >= 0 && idx < SENSOR_ADC_CHANNELS) {
        return (int) idx;
    }

    shell_error(sh, "Unknown channel '%s' (brake, susp_l, susp_r, ntc or 0-3)", arg);
    return -1;
}

/* @return the value, or -1 (with an error printed) if not a number in [0, max] */
static long parse_value(const struct shell *sh, const char *arg, long max) {
    int err = 0;
    long v = shell_strtol(arg, 10, &err);
    if (err != 0 || v < 0 || v > max) {
        shell_error(sh, "'%s' is not a number from 0 to %ld", arg, max);
        return -1;
    }
    return v;
}

static void set_override(const struct shell *sh, int channel, long counts) {
    atomic_set(&override[channel], (atomic_val_t) counts);
    LOG_WRN("%s simulated: %ld counts (%ld mV)", channel_names[channel], counts,
            (counts * VREF_MV + ADC_MAX / 2) / ADC_MAX);
    shell_print(sh, "%s = %ld counts. The filtered value settles in ~100 ms.",
                channel_names[channel], counts);
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);

    int channel = parse_channel(sh, argv[1]);
    long counts = parse_value(sh, argv[2], ADC_MAX);
    if (channel < 0 || counts < 0) {
        return -EINVAL;
    }

    set_override(sh, channel, counts);
    return 0;
}

static int cmd_mv(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);

    int channel = parse_channel(sh, argv[1]);
    long mv = parse_value(sh, argv[2], VREF_MV);
    if (channel < 0 || mv < 0) {
        return -EINVAL;
    }

    set_override(sh, channel, (mv * ADC_MAX + VREF_MV / 2) / VREF_MV);
    return 0;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) {
        for (size_t i = 0; i < SENSOR_ADC_CHANNELS; i++) {
            atomic_set(&override[i], SIM_OFF);
        }
        LOG_INF("Simulation off, all channels back to the real inputs");
        return 0;
    }

    int channel = parse_channel(sh, argv[1]);
    if (channel < 0) {
        return -EINVAL;
    }

    atomic_set(&override[channel], SIM_OFF);
    LOG_INF("%s back to the real input", channel_names[channel]);
    return 0;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct vehicle_sensor_data d = sensors_get();
    const float value[SENSOR_ADC_CHANNELS] = {
        [ADC_BRAKE] = d.rear_brake_bar,
        [ADC_SUSP_LEFT] = d.susp_left_mm,
        [ADC_SUSP_RIGHT] = d.susp_right_mm,
        [ADC_NTC1] = d.ntc1_c,
    };
    static const char *const unit[SENSOR_ADC_CHANNELS] = {
        [ADC_BRAKE] = "bar",
        [ADC_SUSP_LEFT] = "mm",
        [ADC_SUSP_RIGHT] = "mm",
        [ADC_NTC1] = "C",
    };

    shell_print(sh, "channel  source  raw   filtered  pin mV  value");
    for (size_t i = 0; i < SENSOR_ADC_CHANNELS; i++) {
        bool simulated = atomic_get(&override[i]) != SIM_OFF;
        shell_print(sh, "%-7s  %-6s  %4u  %8u  %6u  %.2f %s", channel_names[i],
                    simulated ? "SIM" : "real", d.raw[i], d.filtered[i],
                    (unsigned int) ((d.filtered[i] * VREF_MV + ADC_MAX / 2) / ADC_MAX),
                    (double) value[i], unit[i]);
    }
    return 0;
}

/* Tab completion of the channel names */
static void channel_name_get(size_t idx, struct shell_static_entry *entry) {
    entry->syntax = (idx < SENSOR_ADC_CHANNELS) ? channel_names[idx] : NULL;
    entry->handler = NULL;
    entry->help = NULL;
    entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_channel, channel_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sim,
    SHELL_CMD_ARG(set, &dsub_channel, "Simulate raw ADC counts: sim set <channel> <0-4095>",
                  cmd_set, 3, 0),
    SHELL_CMD_ARG(mv, &dsub_channel, "Simulate the pin voltage: sim mv <channel> <0-3300>",
                  cmd_mv, 3, 0),
    SHELL_CMD_ARG(off, &dsub_channel, "Back to the real input: sim off [channel]",
                  cmd_off, 1, 1),
    SHELL_CMD_ARG(show, NULL, "Override state and the resulting sensor values",
                  cmd_show, 1, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sim, &sub_sim, "Simulate the ADC sensor inputs (brake, susp_l, susp_r, ntc)", NULL);
