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
#include <stdarg.h>
#include <stdio.h>

// Weakly defined logging function - can be overridden by user
__attribute__((weak)) void radio_log(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

#ifndef PHY_RADIO_FRAME_SYNC_LOG_ENABLE
#define PHY_RADIO_FRAME_SYNC_LOG_ENABLE (1)
#endif /* PHY_RADIO_FRAME_SYNC_LOG_ENABLE */

#if PHY_RADIO_FRAME_SYNC_LOG_ENABLE == 1
#define LOG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#define LOG_TIMER_ERROR(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG(f_, ...)
#define LOG_TIMER_ERROR(f_, ...)
#endif /* PHY_RADIO_FRAME_SYNC_LOG_ENABLE */

#ifdef PHY_RADIO_FRAME_SYNC_LOG_DEBUG_ENABLE
#define LOG_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_DEBUG(f_, ...)
#endif /* PHY_RADIO_FRAME_SYNC_LOG_DEBUG_ENABLE */

#ifdef PHY_RADIO_FRAME_SYNC_LOG_V_DEBUG_ENABLE
#define LOG_V_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_V_DEBUG(f_, ...)
#endif /* PHY_RADIO_FRAME_SYNC_LOG_V_DEBUG_ENABLE */

#ifndef MODULO_INC
#define MODULO_INC(value, base) (((value) + 1) % (base))
#endif /* MODULO_INC */

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time);
static int32_t new_frame_callback(phyRadioTimer_t *interface);
static int32_t tick_timer_callback(phyRadioTimer_t *interface, uint16_t index);
static int32_t phyRadioFrameSyncQueueNextSync(phyRadioFrameSync_t *inst, phyRadioPacket_t **sync_packet);

// This timer triggers at the start of a new frame, this marks the start of the frames first guard period where the radio can be managed
static int32_t new_frame_callback(phyRadioTimer_t *interface) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    // Increment the frame counter
    inst->frame_counter = MODULO_INC(inst->frame_counter, inst->sync_interval);

    if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_NEW_FRAME;
    } else {
        // Manage the slot
        int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_NEW_FRAME_EVENT, 0);
        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
            inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
        }

        // Check if we should send sync this frame
        if (inst->frame_counter == 0 && inst->mode == PHY_RADIO_FRAME_SYNC_MODE_CENTRAL) {
            int32_t res = phyRadioFrameSyncQueueNextSync(inst, inst->active_item);
            if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
                LOG("Failed to queue sync %i\n", res);
                inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
                return res;
            }
        }
    }

    return 0;
}

// This timer triggers at every tick interval after the frame timer
static int32_t tick_timer_callback(phyRadioTimer_t *interface, uint16_t frame_index) {
    // Get the current phy radio instance
    phyRadioFrameSync_t *inst = CONTAINER_OF(interface, phyRadioFrameSync_t, radio_timer);

    phyRadioSlotHandlerEvent_t frame_event     = SLOT_HANDLER_ERROR_EVENT;
    uint8_t                    interrupt_event = PHY_RADIO_FRAME_SYNC_INT_IDLE;

    // Figure out where in the slot we are
    switch (frame_index) {
        case PHY_RADIO_TIMER_FIRST_SLOT_GUARD_START: {
            // This marks the start of the first slot
            // Get the current time
            inst->slot_start_time = phyRadioTimerGetTime();
            inst->slot_index = 0;

            frame_event = SLOT_HANDLER_FIRST_SLOT_START_EVENT;
            interrupt_event = PHY_RADIO_FRAME_SYNC_INT_FIRST_SLOT;

            // Check if we should send sync this frame
            if (inst->frame_counter == 0 && inst->mode == PHY_RADIO_FRAME_SYNC_MODE_CENTRAL) {
                inst->pkt_sent_time = inst->slot_start_time;

                // Set the first slot to TX since we have a sync packet queued
                int32_t res = phyRadioSlotHandlerSetSlotCur(inst->slot_handler, PHY_RADIO_CENTRAL_TX_SLOT, PHY_RADIO_SLOT_TX);
                if (res != PHY_RADIO_SLOT_HANDLER_SUCCESS) {
                    LOG("Failed to set slot to TX %i\n", res);
                    inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
                    return res;
                }
            }
        } break;
        case PHY_RADIO_TIMER_SLOT_START: {
            // This markes the end of the guard and start of the active slot part
            frame_event     = SLOT_HANDLER_SLOT_START_EVENT;
            interrupt_event = PHY_RADIO_FRAME_SYNC_INT_SLOT_START;
        } break;
        case PHY_RADIO_TIMER_SLOT_END_GUARD: {
            // This marks the start of the end guard, should allways be processed in interrupt context
            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_SLOT_END_GUARD_EVENT, inst->slot_index);
            if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
                inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
                return res;
            }
        } return true;
        case PHY_RADIO_TIMER_SLOT_GUARD_START: {
            // This marks the start of the slot guard
            inst->slot_start_time = phyRadioTimerGetTime();
            inst->slot_index++;
            frame_event     = SLOT_HANDLER_SLOT_GUARD_EVENT;
            interrupt_event = PHY_RADIO_FRAME_SYNC_INT_SLOT_GUARD;
        } break;
        default:
            // Should never happen
            break;
    }

    if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
        // Radio is busy, process in main context
        inst->timer_interrupt = interrupt_event;
    } else {

        int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, frame_event, inst->slot_index);
        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
            inst->mode = PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR;
        }
    }

    return true;
} 

static int32_t syncWithCentral(phyRadioFrameSync_t *inst, uint64_t toa, uint16_t sync_time) {
    if (inst->mode == PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL) {
        // Calculate the offset between our superslot and the central super slot
        int64_t slot_diff = (int64_t)toa - (int64_t)inst->slot_start_time;
        // Do a sanity check of the diff value
        if (slot_diff < PHY_RADIO_SLOT_TIME_US && slot_diff > -PHY_RADIO_SLOT_TIME_US) {
            // 1) compute the per‐slot normalized error
            float offset_us = (float)(slot_diff);
            float err = (offset_us - (float)sync_time);

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
            LOG_DEBUG("Err %i\n", (int32_t)control);
            LOG_DEBUG("SLOT DURATION:   %i us\n", (int32_t)inst->frame_duration);
        }
    // TODO this else statement could be an elif SCAN to clarify
    } else {
        // We know how long it took to send the sync message, but we need to compensate for the guard infront of it
        sync_time += inst->frame_config->slots[0].slot_start_guard_us;

        // Start combined timer with sync_time as offset
        int32_t res = phyRadioTimerStartCombinedTimer(&inst->radio_timer, tick_timer_callback, new_frame_callback, inst->frame_duration, PHY_RADIO_SLOT_GUARD_TIME_US, sync_time);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 9);
            return res;
        }

        // Notify that a first frame has started
        return phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_START_EVENT, 0);
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

static int32_t phyRadioFrameSyncQueueNextSync(phyRadioFrameSync_t *inst, phyRadioPacket_t **sync_packet) {

    // Queue package in hal layer
    cBufferClear(inst->sync_packet.pkt_buffer); // This is safe, lets not check retvals

    // Create the SYNC packet header
    uint8_t packet_header[2];
    uint8_t phy_pkt_type = PHY_RADIO_PKT_INTERNAL_SYNC << PHY_RADIO_PACKET_TYPE_SHIFT;
    packet_header[0] = (uint8_t)(inst->central_sync_msg_time & PHY_RADIO_SYNC_MSG_TIME_LSB_MASK); // WRITE LSB
    packet_header[1] = phy_pkt_type | (uint8_t)((inst->central_sync_msg_time >> PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT) &
                                                    PHY_RADIO_SYNC_MSG_TIME_MSB_MASK); // Write MSB

#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    if (PHY_RADIO_SYNC_GEN_DATA_SIZE > 0) {
        for (uint32_t i = 0; i < PHY_RADIO_SYNC_GEN_DATA_SIZE; i++) {
            cBufferPrependByte(inst->sync_packet.pkt_buffer, inst->sync_packet_gen_data[i]); // This is safe, lets not check retvals
        }
    }
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

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
    if (bytes_in_packets < (PHY_RADIO_FRAME_SYNC_TX_TIME_SIZE + PHY_RADIO_SYNC_GEN_DATA_SIZE)) {
        return HAL_RADIO_CB_DO_NOTHING;
    }

    // Parse the sync message time
    uint16_t sync_msg_time = cBufferReadByte(phy_packet->pkt_buffer);
    phy_header_msb <<= PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT;
    sync_msg_time |= phy_header_msb;
    sync_msg_time &= PHY_RADIO_SYNC_MSG_TIME_MASK; // Mask out the relevant bits

    int32_t res = syncWithCentral(inst, hal_packet->time, sync_msg_time);

#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    // This is not very important, do it after the synchronization
    for (uint32_t i = 0; i < PHY_RADIO_SYNC_GEN_DATA_SIZE; i++) {
        inst->sync_packet_received_gen_data[i] = cBufferReadByte(phy_packet->pkt_buffer);
    }
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

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
            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_NEW_FRAME_EVENT, inst->slot_index);
            if (res != PHY_RADIO_SLOT_HANDLER_SUCCESS) {
                return res;
            }

            // Check if we should send sync this frame
            if (inst->frame_counter == 0 && inst->mode == PHY_RADIO_FRAME_SYNC_MODE_CENTRAL) {
                // Queue the sync packet
                int32_t res = phyRadioFrameSyncQueueNextSync(inst, inst->active_item);
                if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
                    LOG("Failed to queue sync %i\n", res);
                    return res;
                }
            }

            return res;
        } break;

        case PHY_RADIO_FRAME_SYNC_INT_FIRST_SLOT: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            // Manage the packet send if we could not send it in ISR context
            if (inst->frame_counter == 0 && inst->mode == PHY_RADIO_FRAME_SYNC_MODE_CENTRAL) {
                inst->pkt_sent_time = phyRadioTimerGetTime();
            }

            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_FIRST_SLOT_START_EVENT, inst->slot_index);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_SLOT_GUARD: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_SLOT_GUARD_EVENT, inst->slot_index);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_SLOT_START: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_SLOT_START_EVENT, inst->slot_index);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_SLOT_END_GUARD: {
            inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
            int32_t res = phyRadioSlotHandlerEventManager(inst->slot_handler, SLOT_HANDLER_SLOT_END_GUARD_EVENT, inst->slot_index);
            return res;
        } break;
        case PHY_RADIO_FRAME_SYNC_INT_ERROR:
            return PHY_RADIO_FRAME_SYNC_GEN_ERROR;
        default:
            break;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncSetMode(phyRadioFrameSync_t *inst, phyRadioFrameSyncMode_t mode) {

    switch (mode) {
        case PHY_RADIO_FRAME_SYNC_MODE_IDLE: {
            // Cancel any active timer
            int32_t res = phyRadioTimerCancelAll(&inst->radio_timer);
            if (res != PHY_RADIO_TIMER_SUCCESS) {
                LOG("Timer error! %i\n", res);
                return res;
            }
            inst->mode = mode;
            } break;
        case PHY_RADIO_FRAME_SYNC_MODE_CENTRAL:
             inst->mode = mode;
             return phyRadioTimerStartCombinedTimer(&inst->radio_timer, tick_timer_callback, new_frame_callback, inst->frame_config->frame_length_us, PHY_RADIO_SLOT_GUARD_TIME_US, 0);
        case PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL:
             inst->mode = mode;
             break;
        case PHY_RADIO_FRAME_SYNC_MODE_SCAN:
                inst->mode = mode;
                inst->frame_duration = inst->frame_config->frame_length_us;
                inst->float_frame_duration = (float)inst->frame_config->frame_length_us;

                // Cancel any active timers
                return phyRadioTimerCancelAll(&inst->radio_timer);
            break;
        case PHY_RADIO_FRAME_TRANS_TO_PERIPHERAL:
            // To transition from central to peripheral, make sure that the timer is running
            if (phyRadioTimerIsFrameTimerActive(&inst->radio_timer) == PHY_RADIO_TIMER_ACTIVE) {
                inst->mode = PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL;
            } else {
                return PHY_RADIO_FRAME_SYNC_MODE_ERROR;
            }
            break;
        case PHY_RADIO_FRAME_TRANS_TO_CENTRAL:
            // To transition from peripheral to central, make sure that the timer is running
            if (phyRadioTimerIsFrameTimerActive(&inst->radio_timer) == PHY_RADIO_TIMER_ACTIVE) {
                inst->mode = PHY_RADIO_FRAME_SYNC_MODE_CENTRAL;
            } else {
                return PHY_RADIO_FRAME_SYNC_MODE_ERROR;
            }
            break;
        default:
            break;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncTimeLeftInSlot(phyRadioFrameSync_t *inst, uint8_t slot) {
    if (slot >= inst->frame_config->num_slots) {
        return PHY_RADIO_FRAME_SYNC_SLOT_ERROR;
    }

    // Compute how long we have been in this slot, and check that we have not overflowed
    uint64_t time_in_slot = phyRadioTimerGetTime() - inst->slot_start_time;
    uint16_t slot_length_us = inst->frame_config->slots[slot].slot_length_us;

    if (time_in_slot > slot_length_us) {
        return PHY_RADIO_FRAME_SYNC_FRAME_OVERFLOW;
    }

    // Compute difference between current active slot and the complete length of the slot
    return slot_length_us - time_in_slot;
}

int32_t phyRadioFrameSyncDeInit(phyRadioFrameSync_t *inst) {
    int32_t res = PHY_RADIO_FRAME_SYNC_SUCCESS;

    if ((res = phyRadioTimerDeInit(&inst->radio_timer)) != PHY_RADIO_TIMER_SUCCESS) {
        LOG("Phy radio timer deinit failed %i", res);
    }

    inst->timer_interrupt = PHY_RADIO_FRAME_SYNC_INT_IDLE;
    inst->mode            = PHY_RADIO_FRAME_SYNC_MODE_IDLE;

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
    inst->slot_handler   = init_struct->slot_handler;
    inst->hal_radio_inst = init_struct->hal_radio_inst;
    inst->hal_interface  = init_struct->hal_interface;
    inst->my_address     = init_struct->my_address;
    inst->active_item    = init_struct->active_item;
    inst->mode           = PHY_RADIO_FRAME_SYNC_MODE_IDLE;
    
    // Initialize sync message buffer
    cBufferInit(&inst->sync_message_buf, inst->sync_message_array, sizeof(inst->sync_message_array));
    
    // Initialize sync packet
    inst->sync_packet.pkt_buffer = &inst->sync_message_buf;
    inst->sync_packet.type = PHY_RADIO_PKT_INTERNAL_SYNC;
    inst->sync_packet.addr = PHY_RADIO_BROADCAST_ADDR;
    inst->sync_packet.slot = 0;
    inst->sync_packet._phy_queue_item = NULL;

    // Reset all time keepers
    inst->pkt_sent_time         = 0;
    inst->central_sync_msg_time = 0;
    inst->slot_start_time       = 0;

    // Initialize timer (owned by this module)
    int32_t timer_result = phyRadioTimerInit(&inst->radio_timer, PHY_RADIO_FRAME_TIME_US, PHY_RADIO_SLOT_TIME_US);
    if (timer_result != PHY_RADIO_TIMER_SUCCESS) {
        LOG("Timer init failed: %d\n", timer_result);
        return PHY_RADIO_FRAME_SYNC_GEN_ERROR;
    }

    inst->error_prev = 0.0f;
    inst->integral = 0.0f;

    // Initialize frame counter and sync interval
    inst->frame_counter = 0;
    inst->sync_interval = 1; // Default is to send sync every frame

    inst->frame_config = NULL;

    LOG_DEBUG("Frame sync initialized\n");

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncSetStructure(phyRadioFrameSync_t *inst, phyRadioFrameConfig_t *frame) {
    if (inst == NULL || frame == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

    // Reset the timer
    phyRadioTimerDeInit(&inst->radio_timer);
    phyRadioTimerInit(&inst->radio_timer, PHY_RADIO_FRAME_TIME_US, PHY_RADIO_SLOT_TIME_US);

    // Reset all time keepers
    inst->pkt_sent_time         = 0;
    inst->central_sync_msg_time = 0;
    inst->slot_start_time       = 0;

    // Calculate a best effort sync message time estimate to initialize this value
    inst->central_sync_msg_time = halRadioBitRateToDelayUs(inst->hal_radio_inst, PHY_RADIO_FRAME_SYNC_SYNC_MSG_SIZE);

    inst->mode = PHY_RADIO_FRAME_SYNC_MODE_IDLE;

    // The timer MUST be turned of before changing any parameters
    if (inst->mode != PHY_RADIO_FRAME_SYNC_MODE_IDLE) {
        return PHY_RADIO_FRAME_SYNC_MODE_ERROR;
    }

    uint16_t tick_index = 0;
    uint32_t frame_length_us = 0;
    // THIS IS THE ONLY FUNCTION THAT SHOULD MODIFY THE _frame_ticks parameter
    for (uint32_t i = 0; i < frame->num_slots; i++) {
        inst->_frame_ticks[tick_index] = frame->slots[i].slot_start_guard_us;
        tick_index++;

        // Double check that the slot_length is not shorter than the slot_end_guard that is invalid!
        if (frame->slot_end_guard_us > frame->slots[i].slot_length_us) {
            return PHY_RADIO_FRAME_SYNC_FRAME_ERROR;
        }

        // The slot end guard occurs during the slot_length and does not impact the total length of the slot
        inst->_frame_ticks[tick_index] = frame->slots[i].slot_length_us - frame->slot_end_guard_us;
        tick_index++;
        inst->_frame_ticks[tick_index] = frame->slot_end_guard_us;
        tick_index++;

        // Sum the elements in the frame
        frame_length_us += frame->slots[i].slot_start_guard_us;
        frame_length_us += frame->slots[i].slot_length_us;
        frame_length_us += frame->slots[i].slot_end_guard_us;
    }

    // Add the end guard
    frame_length_us += frame->end_guard;

    // Configure the frame
    frame->frame_length_us = phyRadioFrameConfig(&inst->timer_config, frame_length_us, inst->_frame_ticks, tick_index);

    // Prepare configuration
    inst->frame_duration             = frame->frame_length_us;
    inst->float_frame_duration       = (float)frame->frame_length_us;

    inst->frame_config = frame;

    int32_t res = phyRadioTimerSetConfig(&inst->radio_timer, &inst->timer_config);

    if (res != PHY_RADIO_TIMER_SUCCESS) {
        LOG("Timer config failed: %i\n", res);
        return res;
    }

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncGetStructure(phyRadioFrameSync_t *inst, phyRadioFrameConfig_t **frame) {
    *frame = inst->frame_config;

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncClearCustomData(phyRadioFrameSync_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    for (uint32_t i = 0; i < PHY_RADIO_SYNC_GEN_DATA_SIZE; i++) {
        inst->sync_packet_gen_data[i] = 0;
    }
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncSetCustomData(phyRadioFrameSync_t *inst, uint8_t *data, uint32_t data_size) {
    if (inst == NULL || data_size == 0) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

    if (data_size > PHY_RADIO_SYNC_GEN_DATA_SIZE) {
        return PHY_RADIO_FRAME_SYNC_GEN_ERROR;
    }

#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    for (uint32_t i = 0; i < data_size; i++) {
        inst->sync_packet_gen_data[i] = data[i];
    }
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameGetLatestCustomData(phyRadioFrameSync_t *inst, uint8_t **data) {
    if (inst == NULL || data == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    *data = inst->sync_packet_received_gen_data;
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncSetInterval(phyRadioFrameSync_t *inst, uint16_t sync_interval) {
    if (inst == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

    inst->sync_interval = sync_interval;

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncResetFrameCounter(phyRadioFrameSync_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

    inst->frame_counter = 0;

    return PHY_RADIO_FRAME_SYNC_SUCCESS;
}

int32_t phyRadioFrameSyncGetFrameCount(phyRadioFrameSync_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_FRAME_SYNC_NULL_ERROR;
    }

    return inst->frame_counter;
}