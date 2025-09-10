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

/**
 *  TODO's
 *   - The timer instance management will not work, two modules cannot own it, and container of does not work on pointers
 *   - Verify the num_of_slots * slot duration, this is module should keep track of full frames or some kind of subframe, unclear so far ..
 *   - Obviously check the functionality
 *   - Check how we start the different modes, central and peripheral ..
 *   - How do we go from scan to peripheral ?
 */

static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time);
static int32_t new_frame_callback(phyRadioTimer_t *interface);
static int32_t tick_timer_callback(phyRadioTimer_t *interface);

// This alarm triggers just at the start of a frame, this marks the start of the guard period where the radio can be managed
static int32_t new_frame_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_PREPARE;
    } else {
        int32_t res = phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_PREPARE_EVENT);
        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
            inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
        }
    }

    return 0;
}

// This timer triggers at every tick interval after the frame timer
static int32_t tick_timer_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    // Get the current time
    inst->frame_start_time = time_us_64();

    // Stop the tick timer, for now we only want one callback
    // It is safe to skip the retval check here
    phyRadioTimerStopTickTimer(&inst->radio_timer);

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

static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time) {
    if (inst->mode == PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL) {
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
            inst->frame_duration = (int32_t)inst->float_frame_duration;

            int32_t res = phyRadioTimerUpdateCombinedTimer(&inst->radio_timer, inst->float_frame_duration);
            if (res != PHY_RADIO_TIMER_SUCCESS) {
                LOG_TIMER_ERROR("Timer error %i\n", 6);
                return res;
            }

            LOG_V_DEBUG("Sync time %u, diff %i\n", sync_time, (int32_t)slot_diff);
            LOG("Duriation Error: %i us\n", (int32_t)err);
            LOG_DEBUG("SLOT DURATION:   %i us\n", (int32_t)inst->frame_duration);
        }
    } else {
        // We know how long it took to send the sync message, but we need to compensate for the guard infront of it
        sync_time += PHY_RADIO_GUARD_TIME_US;

        // Start combined timer with sync_time as offset
        int32_t res = phyRadioTimerStartCombinedTimer(&inst->radio_timer, tick_timer_callback, new_frame_callback, inst->frame_duration, PHY_RADIO_GUARD_TIME_US, sync_time);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 9);
            return res;
        }

        // Notify that a first frame has started
        return phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_START_EVENT);
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncQueueNextSync(phyRadioFrameSync_t *inst, phyRadioPacket_t **sync_packet) {

    // Queue package in hal layer
    cBufferClear(inst->sync_packet.pkt_buffer); // This is safe, lets not check retvals

    // Create the SYNC packet header
    uint8_t packet_header[2];
    uint8_t phy_pkt_type = PHY_RADIO_PKT_INTERNAL_SYNC << PHY_RADIO_PACKET_TYPE_SHIFT;
    packet_header[0] = (uint8_t)(inst->central_sync_msg_time & PHY_RADIO_SYNC_MSG_TIME_LSB_MASK); // WRITE LSB
    packet_header[1] = phy_pkt_type | (uint8_t)((inst->central_sync_msg_time >> PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT) &
                                                    PHY_RADIO_SYNC_MSG_TIME_MSB_MASK); // Write MSB

    // Add the phy and sync packet header
    cBufferPrependByte(inst->sync_packet.pkt_buffer, packet_header[0]); // This is safe, lets not check retvals
    cBufferPrependByte(inst->sync_packet.pkt_buffer, packet_header[1]); // This is safe, lets not check retvals
    cBufferPrependByte(inst->sync_packet.pkt_buffer, inst->my_address); // This is safe, lets not check retvals

    // Set the sync packet as active packet
    inst->hal_interface->pkt_buffer = inst->sync_packet.pkt_buffer;

    int32_t res = halRadioQueuePackage(inst->hal_radio_inst,
                                       inst->hal_interface,
                                       inst->sync_packet.addr);

    // Give the caller access to this packet
    *sync_packet = &inst->sync_packet;

    // Should we check the result here or just pass it?
    // Could be usefull if we want to know specifically that it was the sync that failed
    return res;
}

int32_t phyRadioFrameSyncSendNextSync(phyRadioFrameSync_t *inst) {

    // Store the absolute time when this packet tx started
    inst->pkt_sent_time = time_us_64();

    // Trigger send of a queued packet
    int32_t res = halRadioQueueSend(inst->hal_radio_inst, false);

    // Should we check the result here or just pass it?
    // Could be usefull if we want to know specifically that it was the sync that failed
    return res;
}

int32_t phyRadioFrameSyncNotifySyncSent(phyRadioFrameSync_t *inst, halRadioPackage_t* hal_packet) {
    inst->pkt_sent_time = hal_packet->time - inst->pkt_sent_time;
    // If this calculation was correct the send time should never be more than 2ms
    // This protects from errounous behaviour, could perhaps be removed
    if (inst->pkt_sent_time < PHY_RADIO_SYNC_MSG_MAX_TIME) {
        inst->central_sync_msg_time = (uint16_t)inst->pkt_sent_time;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncNewSync(phyRadioFrameSync_t *inst, uint16_t phy_header_msb, phyRadioPacket_t *phy_packet, halRadioPackage_t* hal_packet) {

    int32_t bytes_in_packets = cBufferAvailableForRead(phy_packet->pkt_buffer);

    // Double check that the length of the packet is correct, the address and type has already been read
    if (bytes_in_packets < PHY_RADIO_FRAME_SYNC_TX_TIME_SIZE) {
        return HAL_RADIO_CB_DO_NOTHING;
    }

    // Parse the sync message time
    uint16_t sync_msg_time = cBufferReadByte(phy_packet->pkt_buffer);
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
            int32_t res = phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_NEW_FRAME_EVENT);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_PREPARE: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = phyRadioFrameSyncCallback(inst->phy_radio_inst, FRAME_SYNC_PREPARE_EVENT);
            return res;
        } break;
        default:
            break;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncSetMode(phyRadioFrameSync_t *inst, phyRadioFrameSyncMode_t mode) {

    switch (mode) {
        case PHY_RADIO_FRAME_SYNC_MODE_IDLE:
             inst->mode = mode;
            break;
        case PHY_RADIO_FRAME_SYNC_MODE_CENTRAL:
             inst->mode = mode;
             return phyRadioTimerStartCombinedTimer(&inst->radio_timer, tick_timer_callback, new_frame_callback, PHY_RADIO_SLOT_TIME_US, PHY_RADIO_GUARD_TIME_US, 0);
        case PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL:
             inst->mode = mode;
             break;
        case PHY_RADIO_FRAME_SYNC_MODE_SCAN:
                inst->mode = mode;
                inst->frame_duration = PHY_RADIO_SLOT_TIME_US;
                inst->float_frame_duration = (float)PHY_RADIO_SLOT_TIME_US;

                // Cancel any active timers
                return phyRadioTimerCancelAll(&inst->radio_timer);
            break;
        default:
            break;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncTimeLeftInFrame(phyRadioFrameSync_t *inst) {
    // Compute how long we have been in this slot, and check that we have not overflowed
    uint64_t time_in_slot = time_us_64() - inst->frame_start_time;

    if (time_in_slot > inst->frame_duration - PHY_RADIO_GUARD_TIME_US) {
        return PHY_RADIO_FRAME_SYNC_FRAME_OVERFLOW;
    }

    return inst->frame_duration - time_in_slot - PHY_RADIO_GUARD_TIME_US;
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
    inst->phy_radio_inst = init_struct->phy_radio_inst;
    inst->hal_radio_inst = init_struct->hal_radio_inst;
    inst->hal_interface  = init_struct->hal_interface;
    inst->my_address     = init_struct->my_address;
    inst->mode           = PHY_RADIO_FRAME_SYNC_MODE_IDLE;
    
    // Initialize sync message buffer
    cBufferInit(&inst->sync_message_buf, inst->sync_message_array, sizeof(inst->sync_message_array));
    
    // Initialize sync packet
    inst->sync_packet.pkt_buffer = &inst->sync_message_buf;
    inst->sync_packet.type = PHY_RADIO_PKT_INTERNAL_SYNC;
    inst->sync_packet.addr = PHY_RADIO_BROADCAST_ADDR;
    inst->sync_packet.slot = 0;
    
    // Reset all time keepers
    inst->pkt_sent_time         = 0;
    inst->central_sync_msg_time = 0;
    inst->frame_start_time       = 0;
    inst->frame_duration         = PHY_RADIO_SLOT_TIME_US;
    inst->float_frame_duration   = (float)PHY_RADIO_SLOT_TIME_US;

    inst->error_prev = 0.0f;
    inst->integral = 0.0f;

    inst->inverse_of_num_slots = 1.0/((float)PHY_RADIO_SUPERFRAME_LEN);

    // Calculate a best effort sync message time estimate to initialize this value
    inst->central_sync_msg_time = halRadioBitRateToDelayUs(inst->hal_radio_inst, init_struct->hal_bitrate, PHY_RADIO_FRAME_SYNC_SYNC_MSG_SIZE);

    // Initialize timer (owned by this module)
    int32_t timer_result = phyRadioTimerInit(&inst->radio_timer);
    if (timer_result != PHY_RADIO_TIMER_SUCCESS) {
        LOG("Timer init failed: %d\n", timer_result);
        return PHY_RADIO_FRAME_SYNC_GEN_ERROR;
    }
    
    LOG_DEBUG("Frame sync initialized\n");
    
    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}
