/**
 * @file:       phy_radio_frame_sync.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation of PHY radio frame synchronization layer
 *
 * @license: Apache 2.0
 *
 * Copyright 2025 Lucas Wennerholm
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "phy_radio_frame_sync.h"
#include "string.h"

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

#ifndef LOG_DEBUG
#define LOG_DEBUG(f_, ...)// printf((f_), ##__VA_ARGS__)
#endif

#ifndef LOG_V_DEBUG
#define LOG_V_DEBUG(f_, ...)// printf((f_), ##__VA_ARGS__)
#endif

#ifndef LOG_TIMER_ERROR
#define LOG_TIMER_ERROR(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

static int32_t sync_timer_alarm_callback(phyRadioTimer_t *interface);
static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time);
static int32_t prepare_alarm_callback(phyRadioTimer_t *interface);
static int32_t repeating_timer_callback(phyRadioTimer_t *interface);

// This alarm tirggers just before the start of a frame to give time for preparing any time critical data
static int32_t prepare_alarm_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadio_t *inst = CONTAINER_OF(interface, phyRadio_t, radio_timer);

    if (halRadioCheckBusy(&inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_PREPARE;
    } else {
        int32_t res = phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_PREPARE_EVENT);
        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
            inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
        }
    }

    return 0;
}

// This timer triggers on the start of each new frame
int32_t repeating_timer_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    // Get the current time
    inst->frame_start_time = time_us_64();

    if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        // Set interrupt flag
        // If the radio is busy pass the task to the main context
        inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_NEW_FRAME;
    } else {
        int32_t res = phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_NEW_FRAME_EVENT);
        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
            inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
        }
    }

    return true;
} 

// This alarm is used to synchronize with a central device
static int32_t sync_timer_alarm_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    // Get the current time
    inst->frame_start_time = time_us_64();

    // Start the repeating timer, do as little as possible before this point.
    if (PHY_RADIO_TIMER_SUCCESS != phyRadioTimerStartCombinedTimer(&inst->radio_timer, repeating_timer_callback, prepare_alarm_callback, inst->frame_duration, PHY_RADIO_GUARD_TIME_US)) {
        inst->mode = PHY_RADIO_FRAME_SYNC_MODE_TIMER_ERROR;
        LOG_TIMER_ERROR("Timer error %i\n", 1);
    }

    if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        // Set interrupt flag
        // If the radio is busy pass the task to the main context
        inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_NEW_FRAME;
    } else {
        phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_NEW_FRAME_EVENT);
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time) {
    if (inst->sync_state.mode == PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL) {
        // Calculate the offset between our superslot and the central super slot
        int64_t slot_diff = (int64_t)toa - (int64_t)inst->frame_start_time;
        // Do a sanity check of the diff value
        if (slot_diff < PHY_RADIO_SLOT_TIME_US && slot_diff > -PHY_RADIO_SLOT_TIME_US) {
            // 1) compute the per‐slot normalized error
            float offset_us = (float)slot_diff;
            float err = (offset_us - (float)sync_time) * inst->inverse_of_num_slots;

            // 2) P term
            float P = PHY_RADIO_PID_KP * err;

            // 3) I term
            inst->integral += err;
            float I = PHY_RADIO_PID_KI * inst->integral;

            // 4) D term
            float derivative = err - inst->error_prev;
            float D = PHY_RADIO_PID_KD * derivative;
            inst->error_prev = err;

            // 5) combine & apply
            float control = P + I + D;
            inst->float_frame_duration += control;
            inst->frame_duration = (int32_t)inst->float_slot_duration;

            int32_t res = phyRadioTimerUpdateCombinedTimer(&inst->radio_timer, inst->float_frame_duration);
            if (res != PHY_RADIO_TIMER_SUCCESS) {
                LOG_TIMER_ERROR("Timer error %i\n", 6);
                return res;
            }

            LOG_V_DEBUG("Sync time %u, diff %i\n", sync_time, (int32_t)slot_diff);
            LOG_DEBUG("Duriation Error: %i us\n", (int32_t)err);
            LOG_DEBUG("SLOT DURATION:   %i us\n", (int32_t)inst->frame_duration);
        }
    } else {
        // Compute when the next slot should start
        uint64_t next_slot_start = PHY_RADIO_SLOT_TIME_US - sync_time;

        int32_t res = phyRadioTimerStartSyncTimer(&inst->radio_timer, sync_timer_alarm_callback, next_slot_start);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 9);
            return res;
        }

        // Set the next prepare to trigger 1ms before the slot timer
        next_slot_start -= PHY_RADIO_GUARD_TIME_US;

        res = phyRadioTimerStartSinglePrepareTimer(&inst->radio_timer, prepare_alarm_callback, next_slot_start);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 10);
            return res;
        }

        // Notify that a first frame has started
        return phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_START_EVENT);
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncNewSync(phyRadioFrameSync_t *inst, uint16_t phy_header_msb, phyRadioPacket_t *phy_packet, halRadioPackage_t* hal_packet) {

    // Parse the sync message time
    uint16_t sync_msg_time = cBufferReadByte(interface->pkt_buffer);
    phy_header_msb <<= PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT;
    sync_msg_time |= phy_header_msb;
    sync_msg_time &= PHY_RADIO_SYNC_MSG_TIME_MASK; // Mask out the relevant bits

    int32_t res = syncWithCentral(inst, hal_packet->time, sync_msg_time);

    // TODO should we check result here or just pass it??
    return res;
}

int32_t phyRadioFrameSyncProcess(phyRadioFrameSync_t *inst) {

    if (inst->mode < PHY_RADIO_FRAME_SYNC_MODE_IDLE) {
        // Cancel any active timers, ignore the result
        phyRadioTimerCancelAll(&inst->radio_timer);

        return inst->mode;
    }

    switch(inst->timer_interrupt) {
       case PHY_RADIO_FRAME_SYNC_INT_NEW_FRAME: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = manageRepeatingTimerInterrupt(inst);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_PREPARE: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = managePrepareTimerInterrupt(inst);
            return res;
        } break;
        default:
            break;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncInit(phyRadioFrameSync_t *inst, const phyRadioFrameSyncInit_t *init_struct) {
    if (inst == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }
    
    if (init_struct == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }
    
    if (init_struct->hal_radio_inst == NULL || init_struct->hal_interface == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }
    
    // Clear the instance
    memset(inst, 0, sizeof(phyRadioFrameSync_t));
    
    // Set the hal radio pointers and address
    inst->hal_radio_inst = init_struct->hal_radio_inst;
    inst->hal_interface = init_struct->hal_interface;
    inst->my_address = init_struct->my_address;
    
    // Initialize sync message buffer
    cBufferInit(&inst->sync_message_buf, inst->sync_message_array, sizeof(inst->sync_message_array));
    
    // Initialize sync packet
    inst->sync_packet.pkt_buffer = &inst->sync_message_buf;
    inst->sync_packet.type = PHY_RADIO_PKT_INTERNAL_SYNC;
    inst->sync_packet.addr = PHY_RADIO_BROADCAST_ADDR;
    inst->sync_packet.slot = 0;
    
    // Initialize timer (owned by this module)
    int32_t timer_result = phyRadioTimerInit(&inst->radio_timer);
    if (timer_result != PHY_RADIO_TIMER_SUCCESS) {
        LOG("Timer init failed: %d\n", timer_result);
        return PHY_RADIO_FRAME_SYNC_GEN_ERROR;
    }
    
    LOG_DEBUG("Frame sync initialized\n");
    
    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}
