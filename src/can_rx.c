/*
 * CAN receive: decodes the frames queued by the RX filters of both buses.
 * To read a new frame: add it to can_rx_ids[] and add a case in the handler of its bus.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "autonomous_t26.h"
#include "data_t26.h"

#include "app.h"
#include "watchdog.h"
#include "can_bus.h"
#include "can_rx.h"

LOG_MODULE_REGISTER(can_rx, LOG_LEVEL_INF);

/**
 * Array of read frames from all buses
 * {BUS, ID}
 */
const struct can_rx_id can_rx_ids[] = {
    {CAN_BUS_AUTONOMOUS, AUTONOMOUS_T26_AQT1_FRAME_ID},
};

const size_t can_rx_id_count = ARRAY_SIZE(can_rx_ids);

/* Front brake pressure from 0x710 */
#define FRONT_PRESSURE_TIMEOUT_MS 500 /* 0x710 is only trusted while it keeps arriving */

struct front_brake_data {
    float bar;
    uint16_t coded;    /* FRT_BRK_PRESS as received, 0.1 bar/bit */
    int64_t last_rx_ms;
    uint32_t rx_count; /* 0x710 frames decoded since boot */
    bool received;     /* at least one 0x710 since boot */
};

static struct front_brake_data front_brake;
K_MUTEX_DEFINE(front_brake_lock);

struct front_brake_status get_front_brake_status(void) {
    k_mutex_lock(&front_brake_lock, K_FOREVER);
    struct front_brake_data copy = front_brake;
    k_mutex_unlock(&front_brake_lock);

    int64_t age = copy.received ? (k_uptime_get() - copy.last_rx_ms) : -1;
    bool alive = copy.received && (age <= FRONT_PRESSURE_TIMEOUT_MS);

    return (struct front_brake_status){
        .bar = alive ? copy.bar : 0.0f,
        .coded = copy.coded,
        .alive = alive,
        .age_ms = (age < 0) ? UINT32_MAX : (uint32_t) age,
        .rx_count = copy.rx_count,
    };
}

float get_front_brake_pressure(void) {
    return get_front_brake_status().bar;
}

/* Autonomous bus (CAN1) frames */
static void can_handle_autonomous_rx(const struct can_frame *frame) {
    switch (frame->id) {
        case AUTONOMOUS_T26_AQT1_FRAME_ID: {
            struct autonomous_t26_aqt1_t aqt1;

            if (autonomous_t26_aqt1_unpack(&aqt1, frame->data, can_dlc_to_bytes(frame->dlc)) < 0) {
                return;
            }

            k_mutex_lock(&front_brake_lock, K_FOREVER);
            front_brake.bar = (float) autonomous_t26_aqt1_frt_brk_press_decode(aqt1.frt_brk_press);
            front_brake.coded = aqt1.frt_brk_press;
            front_brake.last_rx_ms = k_uptime_get();
            front_brake.rx_count++;
            front_brake.received = true;
            k_mutex_unlock(&front_brake_lock);
            break;
        }
        default:
            break;
    }
}

/* DATA bus (CAN2) frames: add a case per entry added to can_rx_ids[] */
static void can_handle_data_rx(const struct can_frame *frame) {
    switch (frame->id) {
        default:
            break;
    }
}

#define CAN_RX_WAIT_MS 100 /* longest wait for a frame before checking in with the watchdog */

/**
 * CAN RX Thread: decodes the frames queued by the RX filters of both buses.
 */
static void can_rx_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct can_rx_msg msg;

    while (1) {
        watchdog_checkin(WDOG_CAN_RX);

        /* Bounded wait instead of K_FOREVER: the thread must check in even when no frame arrives */
        if (can_bus_receive(&msg, K_MSEC(CAN_RX_WAIT_MS)) < 0) {
            continue;
        }

        /* Filters are standard-ID data frames only, but stay safe */
        if (msg.frame.flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) {
            continue;
        }

        switch (msg.bus) {
            case CAN_BUS_AUTONOMOUS:
                can_handle_autonomous_rx(&msg.frame);
                break;
            case CAN_BUS_DATA:
                can_handle_data_rx(&msg.frame);
                break;
            default:
                break;
        }
    }
}

K_THREAD_DEFINE(can_rx_tid, STACK_SIZE_DEFAULT, can_rx_thread, NULL, NULL, NULL,
                PRIO_CAN_RX, 0, SYS_FOREVER_MS);

void can_rx_start(void) {
    watchdog_register(WDOG_CAN_RX);
    k_thread_start(can_rx_tid);
}
