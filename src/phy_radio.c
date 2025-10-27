/**
 * @file:       phy_radio.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation Phy radio layer, supporting TDMA and ALOHA
 *
 * @license: Apache 2.0
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

#include "phy_radio.h"
#include "string.h"
#include <stdarg.h>

#define MODULO_INC(value, base) (((value) + 1) % (base))

// Weakly defined logging function - can be overridden by user
__attribute__((weak)) void radio_log(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

#ifndef PHY_RADIO_LOG_ENABLE
#define PHY_RADIO_LOG_ENABLE (1)
#endif /* PHY_RADIO_LOG_ENABLE */

#if PHY_RADIO_LOG_ENABLE == 1
#define LOG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#define LOG_TIMER_ERROR(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG(f_, ...)
#define LOG_TIMER_ERROR(f_, ...)
#endif /* PHY_RADIO_LOG_ENABLE  */

#ifdef PHY_RADIO_LOG_DEBUG_ENABLE
#define LOG_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_DEBUG(f_, ...)
#endif /* PHY_RADIO_LOG_DEBUG_ENABLE */

#ifdef PHY_RADIO_LOG_V_DEBUG_ENABLE
#define LOG_V_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_V_DEBUG(f_, ...)
#endif /* PHY_RADIO_LOG_V_DEBUG_ENABLE */

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

typedef enum {
    PHY_RADIO_INT_IDLE = 0,
    PHY_RADIO_INT_FRAME_GUARD_TIMER, // Currently unused
    PHY_RADIO_INT_FRAME_START_TIMER,
    PHY_RADIO_INT_SLOT_GUARD_START_TIMER, // Currently unused
    PHY_RADIO_INT_SLOT_START_TIMER,
    PHY_RADIO_INT_SCAN_TIMER,
    PHY_RADIO_INT_SEND_TIMER,
} phyRadioInterruptEvent_t;

static int32_t sendDuringSlot(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler, uint8_t slot);
static int32_t queuePopFromSlot(phyRadioTdma_t*     scheduler, uint8_t slot, phyRadioPacket_t **data);
static int32_t queuePeakOnSlot(phyRadioTdma_t* scheduler, uint8_t slot, phyRadioPacket_t**  pkt);
static int32_t queueJustPopFromSlot(phyRadioTdma_t* scheduler, uint8_t slot, phyRadioPacket_t *pkt);
static int32_t scan_timer_alarm_callback(phyRadioTaskTimer_t *interface);
static int32_t send_timer_alarm_callback(phyRadioTaskTimer_t *interface);
static int32_t send_fast_timer_alarm_callback(phyRadioFastTaskTimer_t *interface);
static inline int32_t cancelAllTimers(phyRadio_t *inst);
static int32_t queuePutFirstInSlot(phyRadioTdma_t* scheduler, uint8_t slot, phyRadioPacket_t* packet);
static int32_t manageNewFrameTimerInterrupt(phyRadio_t *inst, uint16_t slot_index);
static int32_t manageNewFrameStartTimerInterrupt(phyRadio_t *inst, uint16_t slot_index);
static int32_t manageSlotStartTimerInterrupt(phyRadio_t *inst, uint16_t slot_index);
static int32_t manageSlotGuardTimerInterrupt(phyRadio_t *inst, uint16_t slot_index);
static int32_t clearAndNotifyPacketQueueInSlot(phyRadio_t *inst, phyRadioTdma_t *scheduler, uint8_t slot);

static int32_t send_timer_alarm_callback(phyRadioTaskTimer_t *interface) {
    // Get the current phy radio instance
    phyRadio_t *inst = CONTAINER_OF(interface, phyRadio_t, task_timer);

    // Set the interrupt flag, and manage this is the main context
    inst->timer_interrupt = PHY_RADIO_INT_SEND_TIMER;

    return PHY_RADIO_SUCCESS;
}

static int32_t send_fast_timer_alarm_callback(phyRadioFastTaskTimer_t *interface) {
    // Get the current phy radio instance
    phyRadio_t *inst = CONTAINER_OF(interface, phyRadio_t, fast_task_timer);
    // Set the interrupt flag, and manage this is the main context
    inst->timer_interrupt = PHY_RADIO_INT_SEND_TIMER;

    return PHY_RADIO_SUCCESS;
}

static int32_t scan_timer_alarm_callback(phyRadioTaskTimer_t *interface) {
    // Get the current phy radio instance
    phyRadio_t *inst = CONTAINER_OF(interface, phyRadio_t, task_timer);

    // Set the interrupt flag, and manage this is the main context
    inst->timer_interrupt = PHY_RADIO_INT_SCAN_TIMER;

    return PHY_RADIO_SUCCESS;
}

// Start of new frame, after this there is a guard time
static int32_t manageNewFrameTimerInterrupt(phyRadio_t *inst, uint16_t slot_index) {
    // Cancel RX
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;
    tdma_scheduler->current_slot = slot_index;

    // Increment the frame counter
    tdma_scheduler->frame_counter = MODULO_INC(tdma_scheduler->frame_counter, tdma_scheduler->sync_interval);

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    // Inidicate new frame by toggling the GPIO
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
            int32_t res = halRadioCancelReceive(&inst->hal_radio_inst);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to cancel %i\n",res);
                return res;
            }

            // Send the sync every sync_interval
            if (tdma_scheduler->frame_counter == 0) {
                // Check if we are the central device and are expected to send a sync
                if (inst->sync_state.mode == PHY_RADIO_MODE_CENTRAL) {
                    int32_t res = phyRadioFrameSyncQueueNextSync(&inst->tdma_scheduler.frame_sync, &inst->tdma_scheduler.active_item);
                    if (res != STATIC_QUEUE_SUCCESS) {
                        LOG("Failed to queue %i\n", res);
                        return res;
                    }

                    // A sync packet is now in fligt
                    inst->tdma_scheduler.in_flight = true;
                } else {
                    // Keep track of number of syncs
                    inst->tdma_scheduler.sync_counter++;
                }
            }
        } break;
        case PHY_RADIO_SLOT_RX: {
            int32_t res = PHY_RADIO_SUCCESS;
            if (inst->tdma_scheduler.in_flight) {
                // A packet failed to complete it's transmission, cancel TX mode.
                if ((res = halRadioCancelTransmit(&inst->hal_radio_inst)) != HAL_RADIO_SUCCESS) {
                    return res;
                }
                // The management of this will be done in main context
            }

            // Go to RX mode
            inst->hal_interface.pkt_buffer = &inst->rx_buffer;
            res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to receive %i\n", res);
                return res;
            }

            if (tdma_scheduler->frame_counter == 0) {
                // Keep track of number of syncs
                inst->tdma_scheduler.sync_counter++;
            }
        } break;
        default:
            // Do nothing
            break;
    }

    // The frame start does not require any main context processing
    return PHY_RADIO_SUCCESS;
}

// Start of first slot, time to send sync
static int32_t manageNewFrameStartTimerInterrupt(phyRadio_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 1);
#endif

            if (tdma_scheduler->frame_counter == 0) {
                // We are expected to send the next sync message
                int32_t res = phyRadioFrameSyncSendNextSync(&inst->tdma_scheduler.frame_sync);

                if (res != HAL_RADIO_SUCCESS) {
                    LOG("Failed to queue send %i\n", res);
                    return res;
                }
            }
        } break;
        case PHY_RADIO_SLOT_RX:
            // If this is the case we are allready in RX mode, lets hope we get a sync
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif
            break;
        default:
            // Do nothing
            break;
    }

    // Set the interrupt flag for next processing
    inst->timer_interrupt = PHY_RADIO_INT_FRAME_START_TIMER;
}

static int32_t manageSlotStartTimerInterrupt(phyRadio_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // Trigger main context processing
    inst->timer_interrupt = PHY_RADIO_INT_SLOT_START_TIMER;

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 1);
#endif
        } break;
        case PHY_RADIO_SLOT_RX:
            // If this is the case we are allready in RX mode, lets hope we get a sync
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif
            break;
        default:
            // Do nothing
            break;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t manageSlotGuardTimerInterrupt(phyRadio_t *inst, uint16_t slot_index) {
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // Store the next slot
    tdma_scheduler->current_slot = slot_index;

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {

            // If the next slot is TX first Cancel RX
            int32_t res = halRadioCancelReceive(&inst->hal_radio_inst);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to cancel %i\n",res);
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX: {

            int32_t res = PHY_RADIO_SUCCESS;
            if (inst->tdma_scheduler.in_flight) {
                // A packet failed to complete it's transmission, cancel TX mode.
                if ((res = halRadioCancelTransmit(&inst->hal_radio_inst)) != HAL_RADIO_SUCCESS) {
                    return res;
                }
                // The management of this will be done in main context
            }

            // Go to RX mode
            inst->hal_interface.pkt_buffer = &inst->rx_buffer;
            res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to receive %i\n", res);
                return res;
            }
        } break;
        default:
            // Do nothing
            break;
    }

    // The start of a guard period in a new slot does not require any main context processing
    return PHY_RADIO_SUCCESS;
}

static int32_t packetTimeEstimate(phyRadio_t *inst, uint8_t num_bytes) {

    int32_t time_us = halRadioBitRateToDelayUs(&inst->hal_radio_inst, PHY_RADIO_BIT_RATE, num_bytes);

    // Check the result, should never fail
    if (time_us < HAL_RADIO_SUCCESS) {
        return time_us;
    }

    // Add the guard time
    time_us += PHY_RADIO_PACKET_GUARD_TIME_US;

    return time_us;
}

static int32_t sendOnTimerInterrupt(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler) {
    int32_t res = PHY_RADIO_SUCCESS;

    // Make sure to remove the new active item from the slot
    if ((res = queueJustPopFromSlot(tdma_scheduler, tdma_scheduler->current_slot, tdma_scheduler->active_item)) != PHY_RADIO_SUCCESS) {
        LOG("Int TX err %i\n", res);
        return res;
    }

    // Lets send this packet!
    LOG_DEBUG("Send after timer interrupt\n");

    phyRadioPacket_t* active_packet = tdma_scheduler->active_item;

    // Assign the current active buffer to the interface
    inst->hal_interface.pkt_buffer = active_packet->pkt_buffer;

    // Get the packet size
    int32_t num_bytes_to_send = 0;
    if ((num_bytes_to_send = cBufferAvailableForRead(tdma_scheduler->active_item->pkt_buffer)) <= 0 || num_bytes_to_send > 255) {
        // The size of the packet cannot be 0
        return PHY_RADIO_BUFFER_ERROR;
    }

    // Calculate how long time it will take for the receiver to read and decode this packet
    int32_t spi_time_us = 0;
    if ((spi_time_us = halRadioSpiDelayEstimateUs(&inst->hal_radio_inst, num_bytes_to_send)) < HAL_RADIO_SUCCESS) {
        return spi_time_us;
    }

    // Store the time
    tdma_scheduler->packet_delay_time_us = spi_time_us;

    // The packet is now in transit
    tdma_scheduler->in_flight = true;
    res = halRadioSendPackageNB(&inst->hal_radio_inst,
                                &inst->hal_interface,
                                active_packet->addr);

    if (res != HAL_RADIO_SUCCESS) {
        LOG_DEBUG("Send Failed\n");
        tdma_scheduler->in_flight        = false;
        inst->tdma_scheduler.active_item = NULL;

        int32_t cb_res = inst->interface->sent_cb(inst->interface, active_packet, res);

        return cb_res;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t sendDuringCb(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler, uint8_t slot) {
    int32_t res = queuePeakOnSlot(tdma_scheduler, slot, &tdma_scheduler->active_item);
    if (res != STATIC_QUEUE_SUCCESS) {
        if (res == STATIC_QUEUE_EMPTY) {
            // Be in standby during empty TX slots
            if ((res = halRadioCancelTransmit(&inst->hal_radio_inst)) != HAL_RADIO_SUCCESS) {
                return res;
            }
            return HAL_RADIO_CB_DO_NOTHING;
        }
        return PHY_RADIO_TDMA_ERROR;
    }

    // There is a packet in the queue, check how large it is and determine if we have time to send it
    int32_t num_bytes_to_send = 0;
    if ((num_bytes_to_send = cBufferAvailableForRead(tdma_scheduler->active_item->pkt_buffer)) <= 0 || num_bytes_to_send > 255) {
        // The size of the packet cannot be 0
        return PHY_RADIO_BUFFER_ERROR;
    }

    int32_t packet_time_us = packetTimeEstimate(inst, (uint8_t)num_bytes_to_send);

    // Calculate how much time we have remaining in the slot excluding the guard period
    int32_t time_remaining_in_slot = phyRadioFrameSyncTimeLeftInSlot(&tdma_scheduler->frame_sync, tdma_scheduler->current_slot);

    // Check the result, negative indicates errors
    if (time_remaining_in_slot < 0) {
        return time_remaining_in_slot;
    }

    // Check if we have time to send it
    if ((packet_time_us + tdma_scheduler->packet_delay_time_us + PHY_RADIO_PACKET_GUARD_TIME_US) > time_remaining_in_slot) {
        // We do not have time for this packet, wait for next slot
        tdma_scheduler->active_item = NULL;

        // Clear the packet queue for this slots and notify that the remaining messages has been canceled
        if ((res = clearAndNotifyPacketQueueInSlot(inst, tdma_scheduler, slot)) != STATIC_QUEUE_SUCCESS) {
            return res;
        }

        return PHY_RADIO_SUCCESS;
    }

    // Make sure that the receiver is done with the previous packet we sent by waiting
    tdma_scheduler->packet_delay_time_us += PHY_RADIO_PACKET_GUARD_TIME_US;

    if (tdma_scheduler->packet_delay_time_us < PHY_RADIO_MAX_BLOCK_DELAY_TIME_US) {
        // If this time is short we can just sleep here.
        sleep_us((uint32_t)tdma_scheduler->packet_delay_time_us);
    } else {
        // If the wait time is long we need to schedule this as a non blocking task

        res = phyRadioTimerCancelFastTaskTimer(&inst->fast_task_timer);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            // Perhaps we need Double check that the alarm instance is not taken, we should not have to cancel the timer here..
            // But sometimes the alarm id is not 0 here, This is very weird and should never happen
            // It would mean that we have an active alarm but still got a packet sent
            // It could mean that the main loop never had time to manage our alarm.
            // Or it could be some other issue where the flag was not correctly set
            // TODO this error can happen but it is fairly rare, try to just cancel the timer, it is probably allready managed
            LOG_TIMER_ERROR("Timer error %i\n", 4);
            return res;
        }

        // Set a fast task timer to trigger the send, if the time is short here is seems like the timer can trigger during this call.
        res = phyRadioTimerStartFastTaskTimer(&inst->fast_task_timer, send_fast_timer_alarm_callback, (uint32_t)tdma_scheduler->packet_delay_time_us);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 5);
            return PHY_RADIO_TIMER_ERROR;
        }

        // Return we should not do anything more here
        return HAL_RADIO_CB_DO_NOTHING;
    }

    // Make sure to remove the new active item from the slot
    if ((res = queueJustPopFromSlot(tdma_scheduler, slot, tdma_scheduler->active_item)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Lets send this packet!
    LOG_DEBUG("Send during CB\n");

    phyRadioPacket_t* active_packet = tdma_scheduler->active_item;

    // Assign the current active buffer to the interface
    inst->hal_interface.pkt_buffer = active_packet->pkt_buffer;

    // Calculate how long time it will take for the receiver to read and decode this packet
    int32_t spi_time_us = 0;
    if ((spi_time_us = halRadioSpiDelayEstimateUs(&inst->hal_radio_inst, num_bytes_to_send)) < HAL_RADIO_SUCCESS) {
        return spi_time_us;
    }

    // Store the time
    tdma_scheduler->packet_delay_time_us = spi_time_us;

    // The packet is now in transit
    tdma_scheduler->in_flight = true;
    res = halRadioSendPackageNB(&inst->hal_radio_inst,
                                &inst->hal_interface,
                                active_packet->addr);

    if (res != HAL_RADIO_SUCCESS) {
        LOG_DEBUG("Send Failed\n");
        tdma_scheduler->in_flight        = false;
        inst->tdma_scheduler.active_item = NULL;

        int32_t cb_res = inst->interface->sent_cb(inst->interface, active_packet, res);

        return cb_res;
    }

    return HAL_RADIO_CB_DO_NOTHING;
}

static int32_t halRadioSentCb(halRadioInterface_t *interface, halRadioPackage_t* hal_packet, halRadioErr_t result) {
    phyRadio_t   *inst = CONTAINER_OF(interface, phyRadio_t, hal_interface);
    phyRadioErr_t send_result = PHY_RADIO_SUCCESS;
    int32_t       cb_res = PHY_RADIO_CB_SUCCESS;

    // Manage the halRadio result
    switch (result) {
        case HAL_RADIO_SEND_FAIL:
            // Manage this specific error
            send_result = PHY_RADIO_SEND_FAIL;
            break;
        case HAL_RADIO_SEND_INTERRUPTED:
            // Manage interrupted transmission, most likely triggered by an inappropriate send and then go to RX by phy
            send_result = PHY_RADIO_SEND_FAIL;

            // Check if we have any active packet in flight, inform higher layer that it has not been sent
            if (inst->tdma_scheduler.in_flight && (inst->tdma_scheduler.active_item->type != PHY_RADIO_PKT_INTERNAL_SYNC)) {
                cb_res = inst->interface->sent_cb(inst->interface, inst->tdma_scheduler.active_item, send_result);
            } else {
                // This would be the sync packet. And it should be fine.
                LOG("Sent but no packet!??\n");
                // This is probably Ok!
                //return PHY_RADIO_GEN_ERROR;
            }

            inst->tdma_scheduler.in_flight   = false;
            inst->tdma_scheduler.active_item = NULL;

            // Ok lets give up, we have failed
            if (cb_res != PHY_RADIO_SUCCESS) {
                return cb_res;
            }

            // Error managed, nothing more to do return
            return HAL_RADIO_CB_SUCCESS;
        default:
            // Just forwade the error
            send_result = result;
            break;
    }

    // Check if we have any active packet in flight, inform higher layer that it has been sent
    if (inst->tdma_scheduler.in_flight && (inst->tdma_scheduler.active_item->type != PHY_RADIO_PKT_INTERNAL_SYNC)) {
        cb_res = inst->interface->sent_cb(inst->interface, inst->tdma_scheduler.active_item, send_result);
    }

    if (cb_res != PHY_RADIO_CB_SUCCESS) {
        LOG("PHY CB FAILED %i\n", cb_res);
        return cb_res;
    }

    switch(inst->sync_state.mode) {
        case PHY_RADIO_MODE_PERIPHERAL:
        case PHY_RADIO_MODE_CENTRAL: {
            // Calculate the time it took to send the sync message, the inflight check protects from null dereference
            if (inst->tdma_scheduler.in_flight && inst->tdma_scheduler.active_item->type == PHY_RADIO_PKT_INTERNAL_SYNC) {

                // Notify frame sync module that the sync has been sent
                int32_t res = phyRadioFrameSyncNotifySyncSent(&inst->tdma_scheduler.frame_sync, hal_packet);
                if (res < PHY_RADIO_SUCCESS) {
                    return res;
                }

                // Notify that a sync has been sent
                res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_SYNC_SENT, &inst->sync_state);
                if (res < HAL_RADIO_CB_SUCCESS) {
                    // Clean up then return error
                    inst->tdma_scheduler.in_flight   = false;
                    inst->tdma_scheduler.active_item = NULL;
                    return res;
                }
            }

            inst->tdma_scheduler.in_flight   = false;
            inst->tdma_scheduler.active_item = NULL;

            // Manage messages in queue
            int32_t res = sendDuringCb(inst, &inst->tdma_scheduler, inst->tdma_scheduler.current_slot);
            if (res < HAL_RADIO_SUCCESS) {
                LOG("Send during sent CB failed %i\n", res);
                return res;
            }
            // Indicate to hal not to change any internal state
            return res;
            } break;
        case PHY_RADIO_MODE_ALOHA: {
            // Return the radio to RX mode
            inst->hal_interface.pkt_buffer = &inst->rx_buffer;
            int32_t res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, true);
            // BUSY is ok, it most likely means a new package has been queued
            if (res != HAL_RADIO_SUCCESS && res != HAL_RADIO_BUSY) {
                LOG("Return to RX failed %i\n", res);
                return res;
            }

            inst->tdma_scheduler.in_flight   = false;
            inst->tdma_scheduler.active_item = NULL;

            // Indicate to hal not to change any internal state
            return HAL_RADIO_CB_DO_NOTHING;
        } break;
        default:
            break;
    }

    return HAL_RADIO_CB_SUCCESS;
}

static int32_t halRadioPackageCb(halRadioInterface_t *interface, halRadioPackage_t* hal_packet) {
    phyRadio_t * inst = CONTAINER_OF(interface, phyRadio_t, hal_interface);

    int32_t bytes_in_packets = cBufferAvailableForRead(interface->pkt_buffer);


    // This was not a phy packet just return
    if (bytes_in_packets < PHY_RADIO_OVERHEAD_SIZE) {
        return HAL_RADIO_CB_DO_NOTHING;
    }

    // These are safe calls since we have checked the validity and size of the buffer
    uint8_t sender_address = cBufferReadByte(interface->pkt_buffer);
    uint16_t phy_header_msb = cBufferReadByte(interface->pkt_buffer); // TODO perhaps we can read both MSB and LSB here??

    // Mask out the packet type
    uint8_t phy_pkt_type = (phy_header_msb & PHY_RADIO_PACKET_TYPE_MASK) >> PHY_RADIO_PACKET_TYPE_SHIFT;

    phyRadioPacket_t new_packet = {
        .pkt_buffer      = interface->pkt_buffer,
        .type            = phy_pkt_type,
        .addr            = sender_address,
        .slot            = inst->tdma_scheduler.current_slot,
        ._phy_queue_item = NULL,
    };

    // Manage what happens next
    switch(inst->sync_state.mode) {
        case PHY_RADIO_MODE_SCAN:
            // Check if the incomming packet is a broadcast, sync packet
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR && phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC) {

                // Central device has been found, synchronize internal timers with central
                int32_t res = phyRadioFrameSyncNewSync(&inst->tdma_scheduler.frame_sync, phy_header_msb, &new_packet, hal_packet);
                if (res != PHY_RADIO_SUCCESS) {
                    LOG("SYNC FAILED! %i\n", res);
                    return res;
                }

                // New sync detected, reset counters
                inst->tdma_scheduler.sync_counter  = 0;
                inst->tdma_scheduler.frame_counter = 0;

                // Switch the phy to peripheral mode, to enable the higher layer to send messages
                inst->sync_state.mode = PHY_RADIO_MODE_PERIPHERAL;

                // A peripheral device must have at least one RX slot to listen on syncs
                inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].main_type    = PHY_RADIO_SLOT_RX;
                inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].current_type = PHY_RADIO_SLOT_RX;

                // Get the custom data sent in the sync packet
                res = phyRadioFrameGetLatestCustomData(&inst->tdma_scheduler.frame_sync, &inst->sync_state.custom_data);
                if (res != PHY_RADIO_SUCCESS) {
                    LOG("SYNC FAILED! %i\n", res);
                    return res;
                }

                // Update frame sync state
                res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL);
                if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
                    return res;
                }

                // Notify that a central device has been found
                inst->sync_state.sync_slot_number = PHY_RADIO_PERIPHERAL_RX_SLOT;
                inst->sync_state.central_address = sender_address;
                int32_t cb_res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_FIRST_SYNC, &inst->sync_state);

                // Manage the callback result
                switch (cb_res) {
                    case PHY_RADIO_CB_SET_PERIPHERAL:
                        // We are allready in peripheral mode
                        break;
                    case PHY_RADIO_CB_SUCCESS:
                        // Return to scan mode
                    case PHY_RADIO_CB_SET_SCAN: {
                        // Reset the sync state
                        inst->sync_state.central_address = 0x00;

                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Return to scan mode
                        if ((res = phyRadioSetScanMode(inst, inst->tdma_scheduler.scan_timeout_ms)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error 1 %i!\n", cb_res);
                        return cb_res;
                }
            }
            // Make sure we stay in RX even if we got something which is not a sync
            return HAL_RADIO_CB_DO_NOTHING;
        case PHY_RADIO_MODE_PERIPHERAL:
            // Check if the incomming packet is a new sync from the current central
            // We can only be synced to one central at a time
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR &&
                phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC) {// &&
                // TODO we allow all centrals here, this could create major issues if the central is out of sync
                //inst->sync_state.central_address == sender_address) {

                // Resync with central 
                int32_t res = phyRadioFrameSyncNewSync(&inst->tdma_scheduler.frame_sync, phy_header_msb, &new_packet, hal_packet);
                if (res != PHY_RADIO_SUCCESS) {
                    LOG("SYNC FAILED! %i\n", res);
                    return res;
                }

                // New superslot detected, new sync detected
                inst->tdma_scheduler.sync_counter  = 0;
                inst->tdma_scheduler.frame_counter = 0;

                // Get the custom data sent in the sync packet
                res = phyRadioFrameGetLatestCustomData(&inst->tdma_scheduler.frame_sync, &inst->sync_state.custom_data);
                if (res != PHY_RADIO_SUCCESS) {
                    LOG("SYNC FAILED! %i\n", res);
                    return res;
                }

                // Notify that a resync has been received
                int32_t cb_res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_RE_SYNC, &inst->sync_state);

                // Manage the callback result
                switch (cb_res) {
                    case PHY_RADIO_CB_SUCCESS:
                    case PHY_RADIO_CB_SET_PERIPHERAL:

                        break;
                    case PHY_RADIO_CB_SET_SCAN: {
                        // Reset the sync state
                        inst->sync_state.central_address = 0x00;

                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Go to scan mode
                        if ((res = phyRadioSetScanMode(inst, inst->tdma_scheduler.scan_timeout_ms)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error 2 %i!\n", cb_res);
                        return cb_res;
                }

                // Notify that the radio should stay in RX mode
                return HAL_RADIO_CB_DO_NOTHING;
            }
            // Fall through
        case PHY_RADIO_MODE_CENTRAL:
            // Check if it is a sync packet from another central
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR && phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC) {

                // Notify that a conflicting sync has been received
                int32_t cb_res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_CONFLICT_SYNC, &inst->sync_state);

                // Manage the callback result
                int32_t res = PHY_RADIO_SUCCESS;
                switch (cb_res) {
                    case PHY_RADIO_CB_SUCCESS:
                        break;
                    case PHY_RADIO_CB_SET_PERIPHERAL: {
                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Switch from Central to peripheral, try to sync
                        int32_t res = phyRadioFrameSyncNewSync(&inst->tdma_scheduler.frame_sync, phy_header_msb, &new_packet, hal_packet);
                        if (res != PHY_RADIO_SUCCESS) {
                            LOG("SYNC FAILED! %i\n", res);
                            return res;
                        }

                        // New superslot detected, new sync detected
                        inst->tdma_scheduler.sync_counter = 0;

                        // Notify that a device has been found
                        inst->sync_state.sync_slot_number = PHY_RADIO_PERIPHERAL_RX_SLOT;
                        inst->sync_state.central_address  = sender_address;

                        // A peripheral device must have at least one RX slot to listen on syncs
                        inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].main_type    = PHY_RADIO_SLOT_RX;
                        inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].current_type = PHY_RADIO_SLOT_RX;

                        // Set peripheral mode
                        inst->sync_state.mode = PHY_RADIO_MODE_PERIPHERAL;

                        res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL);
                        if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
                            return res;
                        }
                    } break;
                    case PHY_RADIO_CB_SET_SCAN: {
                        // Reset the sync state
                        inst->sync_state.central_address = 0x00;
                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Go to scan mode
                        if ((res = phyRadioSetScanMode(inst, inst->tdma_scheduler.scan_timeout_ms)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error 3 %i!\n", cb_res);
                        return cb_res;
                }

                // Notify that the radio should stay in RX mode
                return HAL_RADIO_CB_DO_NOTHING;
            }
            // Fall through
        case PHY_RADIO_MODE_ALOHA: {
            // Notify that a new packet has arrived
            int32_t cb_res = inst->interface->packet_cb(inst->interface, &new_packet);
            // If the callback returns bad values stop and return errors
            if (cb_res != PHY_RADIO_CB_SUCCESS) {
                LOG("PHY CB Error 4 %i!\n", cb_res);
                return cb_res;
            }
            // Notify that the radio should stay in RX mode
            return HAL_RADIO_CB_DO_NOTHING;
        } break;
        default:
            break;
    }

    return HAL_RADIO_CB_SUCCESS;
}

static int32_t queuePeakOnTxQueue(phyRadioTdma_t*     scheduler,
                                  phyRadioPacket_t**  pkt)
{
    // Get the next item from the queue without removing it
    staticQueueItem_t* item;
    int32_t            result = staticQueuePeak(&scheduler->static_queue, &item);

    // Copy the data to the input parameter
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t* queue_item = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        *pkt = queue_item->pkt;
    }

    return result;
}

static int32_t queuePopFromTxQueue(phyRadioTdma_t*     scheduler,
                                   phyRadioPacket_t**  pkt)
{
    // Get the next item from the queue
    staticQueueItem_t* item;
    int32_t            result = staticQueuePop(&scheduler->static_queue, &item);

    // Copy the data to the input parameter
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t* queue_item = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        *pkt = queue_item->pkt;
        queue_item->pkt->_phy_queue_item = NULL;
    }

    return result;
}

static int32_t queuePutInTxQueue(phyRadioTdma_t* scheduler, phyRadioPacket_t*   packet)
{
    // Get the next item to which we want to write data
    staticQueueItem_t* item;
    int32_t            result = staticQueuePut(&scheduler->static_queue, &item);

    // Add write data to the queue item
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t * next = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        next->pkt = packet;
        packet->_phy_queue_item = next;
    }

    return result;
}

static int32_t queuePutInSlot(phyRadioTdma_t*     scheduler,
                              uint8_t             slot,
                              phyRadioPacket_t*   packet)
{
    // Get the next item to which we want to write data
    staticQueueItem_t* item;
    int32_t            result = staticQueuePut(&scheduler->slot[slot].static_queue, &item);

    // Add write data to the queue item
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t * next = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        next->pkt = packet;
        packet->_phy_queue_item = next;
    }

    return result;
}

static int32_t queuePutFirstInSlot(phyRadioTdma_t*     scheduler,
                                   uint8_t             slot,
                                   phyRadioPacket_t*   packet)
{
    // Get relevant item from the queue
    staticQueueItem_t* item;
    int32_t            result = staticQueuePutFirst(&scheduler->slot[slot].static_queue, &item);

    // Copy the data to the queue
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t* next = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        next->pkt = packet;
        packet->_phy_queue_item = next;
    }

    return result;
}

static int32_t queuePopFromSlot(phyRadioTdma_t*     scheduler,
                                uint8_t             slot,
                                phyRadioPacket_t**  pkt)
{
    // Get the next item from the queue
    staticQueueItem_t* item;
    int32_t            result = staticQueuePop(&scheduler->slot[slot].static_queue, &item);

    // Copy the data to the input parameter
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t* queue_item = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        *pkt = queue_item->pkt;
        queue_item->pkt->_phy_queue_item = NULL;
    }

    return result;
}

static int32_t queueJustPopFromSlot(phyRadioTdma_t*   scheduler,
                                    uint8_t           slot,
                                    phyRadioPacket_t *pkt)
{
    // Get the next item from the queue and remove it
    staticQueueItem_t* item;
    int32_t            result = staticQueuePop(&scheduler->slot[slot].static_queue, &item);
    pkt->_phy_queue_item      = NULL;
    return result;
}

static int32_t queuePeakOnSlot(phyRadioTdma_t*     scheduler,
                               uint8_t             slot,
                               phyRadioPacket_t**  pkt)
{
    // Get the next item from the queue witout removing it
    staticQueueItem_t* item;
    int32_t            result = staticQueuePeak(&scheduler->slot[slot].static_queue, &item);

    // Copy the data to the input parameter
    if (result == STATIC_QUEUE_SUCCESS) {
        phyRadioSlotItem_t* queue_item = CONTAINER_OF(item, phyRadioSlotItem_t, node);
        *pkt = queue_item->pkt;
    }

    return result;
}

static int32_t clearAndNotifyPacketQueueInSlot(phyRadio_t *inst, phyRadioTdma_t *scheduler, uint8_t slot) {
    if (slot >= PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    int32_t res = PHY_RADIO_SUCCESS;
    phyRadioPacket_t* phy_packet = NULL;
    while(!staticQueueEmpty(&scheduler->slot[slot].static_queue)) {
        if ((res = queuePopFromSlot(scheduler, slot, &phy_packet)) != STATIC_QUEUE_SUCCESS) {
            if (res == STATIC_QUEUE_EMPTY) {
                break;
            }
            return PHY_RADIO_QUEUE_ERROR;
        }

        // Notify that the packet failed
        if (phy_packet->type != PHY_RADIO_PKT_INTERNAL_SYNC) {
            if ((res = inst->interface->sent_cb(inst->interface, phy_packet, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        }
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t clearAndNotifyPacketQueue(phyRadio_t *inst, phyRadioTdma_t *scheduler) {
    int32_t res = PHY_RADIO_SUCCESS;

    // Check if there is an active packet, if there is cancel it
    if (scheduler->in_flight && (scheduler->active_item != NULL)) {
        // Notify that the active packet failed
        if (scheduler->active_item->type != PHY_RADIO_PKT_INTERNAL_SYNC) {
            if ((res = inst->interface->sent_cb(inst->interface, scheduler->active_item, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        }
    }

    scheduler->in_flight = false;
    scheduler->active_item = NULL;

    // Clear packets in the common queue
    phyRadioPacket_t* phy_packet = NULL;
    while(!staticQueueEmpty(&inst->tdma_scheduler.static_queue)) {
        if ((res = queuePopFromTxQueue(&inst->tdma_scheduler, &phy_packet)) != STATIC_QUEUE_SUCCESS) {
            if (res == STATIC_QUEUE_EMPTY) {
                break;
            }
            return PHY_RADIO_QUEUE_ERROR;
        }

        // Notify that the packet failed
        if (phy_packet->type != PHY_RADIO_PKT_INTERNAL_SYNC) {
            if ((res = inst->interface->sent_cb(inst->interface, phy_packet, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        }
    }

    // Clear the queue of any active messages
    for (uint8_t slot = 0; slot < PHY_RADIO_NUM_SLOTS; slot++) {
        if ((res = clearAndNotifyPacketQueueInSlot(inst, scheduler, slot)) != PHY_RADIO_SUCCESS) {
            return res;
        }
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t resetTdmaScheduler(phyRadioTdma_t *inst) {
    // Reset all counters
    inst->current_slot  = 0;
    inst->sync_counter  = 0;
    inst->in_flight     = false;
    inst->frame_counter = 0;

    // TODO should I reset the frame sync??

    // Initialize all queues
    for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        int32_t res = STATIC_QUEUE_INIT(&inst->slot[i].static_queue, inst->slot[i].items, PHY_RADIO_NUM_ITEMS_SLOTS);
        if (res != STATIC_QUEUE_SUCCESS) {
            return res;
        }

        // Reset all slots to IDLE
        inst->slot[i].main_type    = PHY_RADIO_SLOT_IDLE;
        inst->slot[i].current_type = PHY_RADIO_SLOT_IDLE;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t initTdmaScheduler(phyRadioTdma_t *inst, const phyRadioTdmaInit_t *init_struct) {
    // Reset all counters
    inst->current_slot  = 0;
    inst->sync_counter  = 0;
    inst->in_flight     = false;
    inst->frame_counter = 0;

    inst->phy_radio_inst = init_struct->phy_radio_inst;
    inst->hal_interface  = init_struct->hal_interface;
    inst->hal_radio_inst = init_struct->hal_radio_inst;

    // TODO check all necessary init stuff

    const phyRadioFrameSyncInit_t frame_sync_init = {
        .phy_radio_inst = init_struct->phy_radio_inst,
        .hal_bitrate    = init_struct->hal_bitrate,
        .hal_interface  = init_struct->hal_interface,
        .hal_radio_inst = init_struct->hal_radio_inst,
        .my_address     = init_struct->my_address,
    };

    int32_t res = phyRadioFrameSyncInit(&inst->frame_sync, &frame_sync_init);
    if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
        return res;
    }

    // Init the common TX queue
    res = STATIC_QUEUE_INIT(&inst->static_queue, inst->items, PHY_RADIO_NUM_ITEMS);
    if (res != STATIC_QUEUE_SUCCESS) {
        return res;
    }

    // Initialize all queues
    for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        res = STATIC_QUEUE_INIT(&inst->slot[i].static_queue, inst->slot[i].items, PHY_RADIO_NUM_ITEMS_SLOTS);
        if (res != STATIC_QUEUE_SUCCESS) {
            return res;
        }

        // Initiate all slots as IDLE
        inst->slot[i].main_type    = PHY_RADIO_SLOT_IDLE;
        inst->slot[i].current_type = PHY_RADIO_SLOT_IDLE;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t sendDuringSlot(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler, uint8_t slot) {
    // Check if we are sending any prequeued packages

    if (halRadioGetMode(&inst->hal_radio_inst) != HAL_RADIO_IDLE) {
        // The radio is busy, we should not do anything here, wait for packet sent CB
        return PHY_RADIO_SUCCESS;
    }

    int32_t res = queuePopFromSlot(tdma_scheduler, tdma_scheduler->current_slot, &tdma_scheduler->active_item);
    if (res != STATIC_QUEUE_SUCCESS) {
        if (res == STATIC_QUEUE_EMPTY) {
            // Do nothing
            return PHY_RADIO_SUCCESS;
        }
        return PHY_RADIO_TDMA_ERROR;
    }

    phyRadioPacket_t* active_packet = tdma_scheduler->active_item;

    if (res != HAL_RADIO_SUCCESS) {
        LOG("Cancel Failed\n");
        return res;
    }

    inst->hal_interface.pkt_buffer = active_packet->pkt_buffer;

    int32_t num_bytes_to_send = 0;
    if ((num_bytes_to_send = cBufferAvailableForRead(active_packet->pkt_buffer)) <= C_BUFFER_SUCCESS || num_bytes_to_send > 255) {
        return PHY_RADIO_BUFFER_ERROR;
    }

    int32_t spi_time_us = 0;
    if ((spi_time_us = halRadioSpiDelayEstimateUs(&inst->hal_radio_inst, num_bytes_to_send)) < HAL_RADIO_SUCCESS) {
        return spi_time_us;
    }

    // Store the expected delay time for this packet
    tdma_scheduler->packet_delay_time_us = spi_time_us;

    tdma_scheduler->in_flight = true;
    res = halRadioSendPackageNB(&inst->hal_radio_inst,
                                &inst->hal_interface,
                                active_packet->addr);

    if (res != HAL_RADIO_SUCCESS) {
        LOG_DEBUG("Send Failed\n");
        tdma_scheduler->in_flight        = false;
        inst->tdma_scheduler.active_item = NULL;

        int32_t cb_res = inst->interface->sent_cb(inst->interface, active_packet, res);

        return cb_res;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t processCentral(phyRadio_t *inst) {
    phyRadioTdma_t* tdma_scheduler = &inst->tdma_scheduler;
    int32_t res = PHY_RADIO_SUCCESS;

    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
            // Send the next package
            if ((res = sendDuringSlot(inst, tdma_scheduler, tdma_scheduler->current_slot)) != PHY_RADIO_SUCCESS) {
                return res;
            }

            // Notify that a new TX slot has started
            if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_TX_SLOT_START, &inst->sync_state)) < PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX:
            // Check if there is a packet in flight
            if (inst->tdma_scheduler.in_flight) {
                // If there is an active packet in flight when RX starts something has gone wrong.
                // The reasons could be, packet was longer than the slot and got interrupted
                // A sent interrupt got lost
                if (inst->tdma_scheduler.active_item->type == PHY_RADIO_PKT_INTERNAL_SYNC) {
                    // TODO what should we do about this
                    LOG("SYNC MESSAGE SEND FAILED\n");
                } else {
                    if ((res = inst->interface->sent_cb(inst->interface, inst->tdma_scheduler.active_item, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
                        return res;
                    }
                }
                inst->tdma_scheduler.in_flight   = false;
                inst->tdma_scheduler.active_item = NULL;
            }

            // Notify that a new RX slot has started
            if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_RX_SLOT_START, &inst->sync_state)) < PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        break;
        default:
            return PHY_RADIO_SUCCESS;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t processScan(phyRadio_t *inst) {
    int32_t res = PHY_RADIO_SUCCESS;

    // The only way we end up here is if the scan timed out, notify
    if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_SCAN_TIMEOUT, &inst->sync_state)) < PHY_RADIO_SUCCESS) {
        return res;
    }

    return PHY_RADIO_SUCCESS;
}

static inline int32_t cancelAllTimers(phyRadio_t *inst) {
    int32_t res = phyRadioTimerCancelTaskTimer(&inst->task_timer);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        return res;
    }

    return phyRadioTimerCancelFastTaskTimer(&inst->fast_task_timer);
}

static int32_t processPeripheral(phyRadio_t *inst) {
    phyRadioTdma_t* tdma_scheduler = &inst->tdma_scheduler;

    if (tdma_scheduler->sync_counter > PHY_RADIO_SYNC_TIMEOUT) {
        int32_t result = PHY_RADIO_SUCCESS;
        // Notify that sync has been lost
        int32_t cb_res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_SYNC_LOST, &inst->sync_state);

        // Manage the callback result
        switch (cb_res) {
            case PHY_RADIO_CB_SET_SCAN: {
                // Reset the sync state
                inst->sync_state.central_address = 0x00;

                // Set scan mode
                if ((result = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_SYNC_MODE_SCAN)) != PHY_RADIO_SUCCESS) {
                    return result;
                }

                // Go to scan mode
                if ((result = phyRadioSetScanMode(inst, inst->tdma_scheduler.scan_timeout_ms)) != PHY_RADIO_SUCCESS) {
                    return result;
                }
            } break;
            case PHY_RADIO_CB_SET_PERIPHERAL:
                // Stay in peripheral mode
                break;
            case PHY_RADIO_CB_SUCCESS:
                break;
            default:
                LOG("PHY CB Error 5 %i!\n", cb_res);
                return cb_res;
        }

        return PHY_RADIO_SUCCESS;
    }

    int32_t res = PHY_RADIO_SUCCESS;
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
            // Send the next package
            if ((res = sendDuringSlot(inst, tdma_scheduler, tdma_scheduler->current_slot)) != PHY_RADIO_SUCCESS) {
                return res;
            }

            // Notify that a new TX slot has started
            if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_TX_SLOT_START, &inst->sync_state)) < PHY_RADIO_SUCCESS) {
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX: {
            // Check if there is a packet in flight
            if (inst->tdma_scheduler.in_flight) {
                // If there is an active packet in flight when RX starts something has gone wrong.
                // The reasons could be, packet was longer than the slot and got interrupted
                // A sent interrupt got lost
                if (inst->tdma_scheduler.active_item->type == PHY_RADIO_PKT_INTERNAL_SYNC) {
                    // TODO what should we do about this
                    LOG("SYNC MESSAGE SEND FAILED\n");
                } else {
                    // External packet lost
                    if ((res = inst->interface->sent_cb(inst->interface, inst->tdma_scheduler.active_item, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
                        return res;
                    }
                }
                inst->tdma_scheduler.in_flight   = false;
                inst->tdma_scheduler.active_item = NULL;
            }

            // Notify that a new TX slot has started
            if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_RX_SLOT_START, &inst->sync_state)) < PHY_RADIO_SUCCESS) {
                return res;
            }
        } break;
        default:
            return PHY_RADIO_SUCCESS;
    }
    return PHY_RADIO_SUCCESS;
}

static int32_t manageStartSyncEvent(phyRadio_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);
    // New sync detected, reset the counters
    inst->tdma_scheduler.current_slot = 0;

    // TODO not sure if this really is necessary..
    // Set the mandatory peripheral sync RX slot
    inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].main_type    = PHY_RADIO_SLOT_RX;
    inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].current_type = PHY_RADIO_SLOT_RX;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioFrameSyncCallback(phyRadio_t *inst, phyRadioFrameSyncEvent_t event, uint16_t slot_index) {
    // Manage the new event
    switch (event)
    {
        case FRAME_SYNC_START_EVENT:
            return manageStartSyncEvent(inst, slot_index);
        case FRAME_SYNC_NEW_FRAME_EVENT:
            return manageNewFrameTimerInterrupt(inst, slot_index);
        case FRAME_SYNC_FIRST_SLOT_START_EVENT:
            return manageNewFrameStartTimerInterrupt(inst, slot_index);
        case FRAME_SYNC_SLOT_GUARD_EVENT:
            return manageSlotGuardTimerInterrupt(inst, slot_index);
        case FRAME_SYNC_SLOT_START_EVENT:
            return manageSlotStartTimerInterrupt(inst, slot_index);
        case FRAME_SYNC_ERROR_EVENT:
            // Fall through
        default:
            inst->sync_state.mode = PHY_RADIO_MODE_FRAME_ERROR;
            break;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioEventInQueue(phyRadio_t *inst) {
    if (halRadioEventInQueue(&inst->hal_radio_inst) > 0) {
        return PHY_RADIO_INTERRUPT_IN_QUEUE;
    }

    if (inst->timer_interrupt > 0) {
        return PHY_RADIO_INTERRUPT_IN_QUEUE;
    }

    return PHY_RADIO_SUCCESS;
}

static int32_t phyRadioManageFrameStart(phyRadio_t *inst) {
    int32_t res = PHY_RADIO_SUCCESS;

    // Loop over all slots and mange their current type configuraion
    for (int i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        // If it was a temporary type set it back to main type
        if (inst->tdma_scheduler.slot[i].current_type != inst->tdma_scheduler.slot[i].main_type) {
            inst->tdma_scheduler.slot[i].current_type = inst->tdma_scheduler.slot[i].main_type;
        }
    }

    phyRadioPacket_t* phy_packet = NULL;
    // Move packets from the common queue to the slot queues
    while(!staticQueueEmpty(&inst->tdma_scheduler.static_queue)) {
        if ((res = queuePopFromTxQueue(&inst->tdma_scheduler, &phy_packet)) != STATIC_QUEUE_SUCCESS) {
            if (res == STATIC_QUEUE_EMPTY) {
                break;
            }
            return PHY_RADIO_QUEUE_ERROR;
        }

        // Set this slot as an active TX slot
        inst->tdma_scheduler.slot[phy_packet->slot].current_type = PHY_RADIO_SLOT_TX;

        if ((res = queuePutInSlot(&inst->tdma_scheduler, phy_packet->slot, phy_packet)) != STATIC_QUEUE_SUCCESS) {
            return res;
        }
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioProcess(phyRadio_t *inst) {
    switch(inst->timer_interrupt) {
        case PHY_RADIO_INT_IDLE:
            break;
        case PHY_RADIO_INT_FRAME_START_TIMER: {
            // Main context processing of a frame start
            inst->timer_interrupt = PHY_RADIO_INT_IDLE;
            int32_t res = PHY_RADIO_SUCCESS;
            if ((res = phyRadioManageFrameStart(inst)) != PHY_RADIO_SUCCESS) {
                return res;
            }

            switch(inst->sync_state.mode) {
                case PHY_RADIO_MODE_CENTRAL:
                    if ((res = processCentral(inst)) != PHY_RADIO_SUCCESS) {
                        return res;
                    }
                    break;
                case PHY_RADIO_MODE_PERIPHERAL:
                    if ((res = processPeripheral(inst)) != PHY_RADIO_SUCCESS) {
                        return res;
                    }
                    break;
                default:
                    break;
            }

            // Notify that a new frame has started
            if ((res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_FRAME_START, &inst->sync_state)) < PHY_RADIO_CB_SUCCESS) {
                        LOG("proc fail 4 %i\n", res);
                return res;
            }
        } break;
        case PHY_RADIO_INT_SLOT_START_TIMER:
            // Main context processing of a slot start
            inst->timer_interrupt = PHY_RADIO_INT_IDLE;
            switch(inst->sync_state.mode) {
                case PHY_RADIO_MODE_CENTRAL:
                    return processCentral(inst); // This is the start of a regular slot
                case PHY_RADIO_MODE_PERIPHERAL:
                    return processPeripheral(inst);
                default:
                    break;
            }

            break;
        case PHY_RADIO_INT_SCAN_TIMER: {
            // Manage timer tasks
            inst->timer_interrupt = PHY_RADIO_INT_IDLE;
            LOG_V_DEBUG("Repeat at %lld\n", time_us_64());

            switch(inst->sync_state.mode) {
                case PHY_RADIO_MODE_SCAN:
                    return processScan(inst);
                default:
                    if (inst->sync_state.mode < PHY_RADIO_MODE_IDLE) {
                        // Cancel any active timers, ignore the result
                        cancelAllTimers(inst);
                        return inst->sync_state.mode;
                    }
                    break;
            }
        } break;
        case PHY_RADIO_INT_SEND_TIMER: {
            // Manage send task
            inst->timer_interrupt = PHY_RADIO_INT_IDLE;
            int32_t res = sendOnTimerInterrupt(inst, &inst->tdma_scheduler);

            return res;
        } break;
        default:
            break;
    }

    int32_t res = phyRadioFrameSyncProcess(&inst->tdma_scheduler.frame_sync);
    if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
        return res;
    }

    res = halRadioProcess(&inst->hal_radio_inst);

    switch (res) {
        case HAL_RADIO_RECEIVE_FAIL:
            // This indicates that a bad interrupt occured
            LOG("Hal Radio Receive failed\n");
            // Just restart radio reception
            res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, true);
            break;
        default:
            break;
    }

    return res;
}

int32_t phyRadioInit(phyRadio_t *inst, phyRadioInterface_t *interface, uint8_t address) {

    if (inst == NULL || interface == NULL || interface->sent_cb == NULL || interface->packet_cb == NULL || interface->sync_state_cb == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Init the halRadio and set the receiver address
    halRadioConfig_t hal_config = {
        .bitrate = PHY_RADIO_BIT_RATE,
        .broadcast_address = PHY_RADIO_BROADCAST_ADDR,
        .rx_address = address,
        .channel    = PHY_RADIO_DEFAULT_CHANNEL,
        .power_dbm  = PHY_RADIO_DEFAULT_TX_POWER_DBM,
    };

    int32_t res = halRadioInit(&inst->hal_radio_inst, hal_config);
    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    // Lets make sure that the longest packets supported fit in our slot configuration
    int32_t packet_time_us = packetTimeEstimate(inst, PHY_RADIO_MAX_PACKET_SIZE);
    if (packet_time_us < PHY_RADIO_SUCCESS) {
        // An unkonwn error occured
        return packet_time_us;
    }

    if (packet_time_us > PHY_RADIO_SLOT_TIME_US) {
        return PHY_RADIO_INVALID_SLOT;
    }

    inst->hal_interface.package_cb  = halRadioPackageCb;
    inst->hal_interface.pkg_sent_cb = halRadioSentCb;

    inst->interface  = interface;
    inst->my_address = address;

    const phyRadioTdmaInit_t tdma_init = {
        .phy_radio_inst = inst,
        .hal_bitrate    = hal_config.bitrate,
        .hal_interface  = &inst->hal_interface,
        .hal_radio_inst = &inst->hal_radio_inst,
        .my_address     = inst->my_address,
    };

    res = initTdmaScheduler(&inst->tdma_scheduler, &tdma_init);
    if (res != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Initialize task timer AFTER TDMA scheduler (which sets up PWM IRQ handler)
    res = phyRadioTaskTimerInit(&inst->task_timer);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        return res;
    }

    // Initialize fast task timer AFTER TDMA scheduler (which sets up PWM IRQ handler)
    res = phyRadioFastTaskTimerInit(&inst->fast_task_timer);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        return res;
    }

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    // Init the TX RX GPIO
    gpio_init(HAL_RADIO_PIN_TX_RX);
    gpio_set_dir(HAL_RADIO_PIN_TX_RX, GPIO_OUT);
#endif

    inst->tdma_scheduler.scan_timeout_ms = 0;

    // Init the RX buffer
    if(cBufferInit(&inst->rx_buffer, inst->rx_byte_array, PHY_RADIO_RX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return PHY_RADIO_GEN_ERROR;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetScanMode(phyRadio_t *inst, uint32_t timeout_ms) {
    if ((inst == NULL) || (inst->interface == NULL)) {
        return PHY_RADIO_NULL_ERROR;
    }

    int32_t res = PHY_RADIO_SUCCESS;
    // Cancel any active timers
    if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Clear the packet queue in all slots and notify that the message has been canceled
    if ((res = clearAndNotifyPacketQueue(inst, &inst->tdma_scheduler)) != STATIC_QUEUE_SUCCESS) {
        return res;
    }

    // Reset the TdmaScheduler
    if ((res = resetTdmaScheduler(&inst->tdma_scheduler)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Scan for the broadcast address
    inst->hal_interface.pkt_buffer = &inst->rx_buffer;
    res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, true);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    if (timeout_ms > 0) {
        uint32_t timeout_us = timeout_ms * 1000; // Convert ms to us
        res = phyRadioTimerStartTaskTimer(&inst->task_timer, scan_timer_alarm_callback, timeout_us);
        if (res != PHY_RADIO_TIMER_SUCCESS) {
            LOG_TIMER_ERROR("Timer error %i\n", 13);
            return PHY_RADIO_TIMER_ERROR;
        }
    }

    inst->tdma_scheduler.scan_timeout_ms = timeout_ms;

    inst->sync_state.mode = PHY_RADIO_MODE_SCAN;

    res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_SYNC_MODE_SCAN);
    if (res != PHY_RADIO_FRAME_SYNC_SUCCESS) {
        return res;
    }

    // Reset all counters
    inst->tdma_scheduler.current_slot = 0;
    inst->tdma_scheduler.sync_counter = 0;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioTransitionPeripheralToCentral(phyRadio_t *inst) {
    // The central device allways occupies one slot as TX to send syncs
    inst->tdma_scheduler.slot[PHY_RADIO_CENTRAL_TX_SLOT].main_type    = PHY_RADIO_SLOT_TX;
    inst->tdma_scheduler.slot[PHY_RADIO_CENTRAL_TX_SLOT].current_type = PHY_RADIO_SLOT_TX;

    inst->sync_state.sync_slot_number = PHY_RADIO_CENTRAL_TX_SLOT;
    inst->sync_state.central_address = inst->my_address;

    // Start broadcasting a sync message to enable other units to adjust their clocks
    int32_t res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_TRANS_TO_CENTRAL);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        LOG_TIMER_ERROR("Frame sync Error %i\n", res);
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_CENTRAL;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioTransitionCentralToPeripheral(phyRadio_t *inst, uint8_t new_central_addr) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // We can only transition if we are in the Central mode
    if (inst->sync_state.mode != PHY_RADIO_MODE_CENTRAL) {
        return PHY_RADIO_INVALID_MODE;
    }


    // A peripheral device allways occupies one slot as RX to receive syncs
    inst->tdma_scheduler.slot[PHY_RADIO_PERIPHERAL_RX_SLOT].main_type = PHY_RADIO_SLOT_RX;

    inst->sync_state.sync_slot_number = PHY_RADIO_PERIPHERAL_RX_SLOT;
    inst->sync_state.central_address = new_central_addr;

    // Change the frame sync mode
    int32_t res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_TRANS_TO_PERIPHERAL);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        LOG_TIMER_ERROR("Frame sync Error %i\n", res);
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_PERIPHERAL;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetCentralMode(phyRadio_t *inst) {
    if ((inst == NULL) || (inst->interface == NULL)) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Cancel any active timers
    int32_t res = PHY_RADIO_SUCCESS;
    if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Clear the packet queue in all slots and notify that the message has been canceled
    if ((res = clearAndNotifyPacketQueue(inst, &inst->tdma_scheduler)) != STATIC_QUEUE_SUCCESS) {
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_CENTRAL;

    if ((res = resetTdmaScheduler(&inst->tdma_scheduler)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // The central device allways occupies one slot as TX to send syncs
    inst->tdma_scheduler.slot[PHY_RADIO_CENTRAL_TX_SLOT].main_type    = PHY_RADIO_SLOT_TX;
    inst->tdma_scheduler.slot[PHY_RADIO_CENTRAL_TX_SLOT].current_type = PHY_RADIO_SLOT_TX;

    inst->sync_state.sync_slot_number = PHY_RADIO_CENTRAL_TX_SLOT;
    inst->sync_state.central_address = inst->my_address;

    // Start broadcasting a sync message to enable other units to adjust their clocks
    res = phyRadioFrameSyncSetMode(&inst->tdma_scheduler.frame_sync, PHY_RADIO_FRAME_SYNC_MODE_CENTRAL);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        LOG_TIMER_ERROR("Frame sync Error %i\n", res);
        return res;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetAlohaMode(phyRadio_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Cancel any active timers
    int32_t res = PHY_RADIO_SUCCESS;
    if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    inst->hal_interface.pkt_buffer = &inst->rx_buffer;
    res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface, true);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_ALOHA;

    // TODO set frame sync to relevant value

    return PHY_RADIO_SUCCESS;
}

static inline int32_t sendAloha(phyRadio_t *inst, phyRadioPacket_t* packet) {
    int32_t res = halRadioCancelReceive(&inst->hal_radio_inst);

    if (res != HAL_RADIO_SUCCESS) {
        LOG("Cancel Failed\n");
        return res;
    }

    // Prepend the packet type
    uint8_t phy_pkt_type = packet->type << PHY_RADIO_PACKET_TYPE_SHIFT;
    if ((res = cBufferPrependByte(packet->pkt_buffer, phy_pkt_type)) != PHY_RADIO_SENDER_ADDR_SIZE) {
        return PHY_RADIO_BUFFER_ERROR;
    }

    // Prepend my address
    if ((res = cBufferPrependByte(packet->pkt_buffer, inst->my_address)) != PHY_RADIO_SENDER_ADDR_SIZE) {
        return PHY_RADIO_BUFFER_ERROR;
    }

    // If the packet is an broadcast packet use the broadcast addr
    if (packet->type == PHY_RADIO_PKT_BROADCAST) {
        packet->addr = PHY_RADIO_BROADCAST_ADDR;
    }

    // Send the new packet
    inst->hal_interface.pkt_buffer = packet->pkt_buffer;
    res = halRadioSendPackageNB(&inst->hal_radio_inst,
                                &inst->hal_interface,
                                packet->addr);

    if (res != HAL_RADIO_SUCCESS) {
        LOG("Send Failed\n");
        return res;
    }

    inst->tdma_scheduler.active_item = packet;
    inst->tdma_scheduler.in_flight   = true;

    return PHY_RADIO_SUCCESS;
}

static inline int32_t sendCentral(phyRadio_t *inst, phyRadioPacket_t* packet) {
    // Check that the slot is valid, protects agains oob
    if (packet->slot >= PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    // Prepend the packet type
    uint8_t phy_pkt_type = packet->type << PHY_RADIO_PACKET_TYPE_SHIFT;
    int32_t res = PHY_RADIO_SUCCESS;
    if ((res = cBufferPrependByte(packet->pkt_buffer, phy_pkt_type)) != PHY_RADIO_SENDER_ADDR_SIZE) {
        return PHY_RADIO_BUFFER_ERROR;
    }

    // Prepend my address
    if ((res = cBufferPrependByte(packet->pkt_buffer, inst->my_address)) != PHY_RADIO_SENDER_ADDR_SIZE) {
        return PHY_RADIO_BUFFER_ERROR;
    }

    // If the packet is an broadcast packet use the broadcast addr
    if (packet->type == PHY_RADIO_PKT_BROADCAST) {
        packet->addr = PHY_RADIO_BROADCAST_ADDR;
    }

    // Store the packet in the common TX queue
    res = queuePutInTxQueue(&inst->tdma_scheduler, packet);
    if (res != STATIC_QUEUE_SUCCESS) {
        return PHY_RADIO_QUEUE_ERROR;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSendOnSlot(phyRadio_t *inst, phyRadioPacket_t* packet) {
    if (inst == NULL || packet == NULL || packet->pkt_buffer == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Manage send given current mode
    switch (inst->sync_state.mode) {
        case PHY_RADIO_MODE_ALOHA:
            return sendAloha(inst, packet);
        case PHY_RADIO_MODE_PERIPHERAL:
        case PHY_RADIO_MODE_CENTRAL:
            return sendCentral(inst, packet);
        case PHY_RADIO_MODE_SCAN:
            return PHY_RADIO_INVALID_MODE;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetSlotIdle(phyRadio_t *inst, uint8_t slot) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Check if the slot is a valid value
    if (slot > PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // Configure the slot
    tdma_scheduler->slot[slot].main_type = PHY_RADIO_SLOT_IDLE;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioReceiveOnSlot(phyRadio_t *inst, uint8_t slot) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Check if the slot is a valid value
    if (slot > PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // Configure the slot
    tdma_scheduler->slot[slot].main_type = PHY_RADIO_SLOT_RX;

    return PHY_RADIO_SUCCESS;
}

static int32_t clearSlotCallback(staticQueue_t *queue, staticQueueItem_t *item) {
    phyRadioSlotItem_t* slot_item = CONTAINER_OF(item, phyRadioSlotItem_t, node);
    phyRadioPacket_t* phy_packet = slot_item->pkt;

    // Get scheduler from the queue
    phyRadioTdma_t *scheduler = CONTAINER_OF(queue, phyRadioTdma_t, static_queue);

    // Check if this packet is for the slot we're clearing
    if (phy_packet->slot == scheduler->fe_slot_target) {
        // Get inst only when needed for callback
        phyRadio_t *inst = CONTAINER_OF(scheduler, phyRadio_t, tdma_scheduler);

        // Clear the back-reference
        phy_packet->_phy_queue_item = NULL;

        // Notify that the packet send failed
        if (phy_packet->type != PHY_RADIO_PKT_INTERNAL_SYNC) {
            int32_t res = inst->interface->sent_cb(inst->interface, phy_packet, PHY_RADIO_SEND_FAIL);
            if (res != PHY_RADIO_CB_SUCCESS) {
                return res;
            }
        }

        // Erase this packet from the queue
        return STATIC_QUEUE_CB_ERASE;
    }

    return STATIC_QUEUE_CB_NEXT;
}

int32_t phyRadioClearSlot(phyRadio_t *inst, uint8_t slot) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    if (slot >= PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    phyRadioTdma_t *scheduler = &inst->tdma_scheduler;
    int32_t res = PHY_RADIO_SUCCESS;

    // Store the slot number in the scheduler for the callback
    scheduler->fe_slot_target = slot;

    // Iterate through common queue and erase packets for this slot
    if ((res = staticQueueForEach(&scheduler->static_queue, clearSlotCallback)) != STATIC_QUEUE_SUCCESS) {
        return res;
    }

    // Clear the slot queue and inform higher layers that the packets are lost
    return clearAndNotifyPacketQueueInSlot(inst, scheduler, slot);
}

int32_t phyRadioSetFrameStructure(phyRadio_t *inst, phyRadioFrameConfig_t *frame) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    if (frame->sync_interval < 1) {
        return PHY_RADIO_FRAME_SYNC_SLOT_ERROR;
    }

    inst->tdma_scheduler.sync_interval = frame->sync_interval;

    return phyRadioFrameSyncSetStructure(&inst->tdma_scheduler.frame_sync, frame);
}

int32_t phyRadioRemoveFromSlot(phyRadio_t *inst, phyRadioPacket_t *pkt) {
    if (inst == NULL || pkt == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    if (pkt->slot >= PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    phyRadioTdma_t *scheduler = &inst->tdma_scheduler;

    // Check if the packet is even in a queue
    if (pkt->_phy_queue_item == NULL) {
        // Packet is not queued
        return PHY_RADIO_SUCCESS;
    }

    // TODO perhaps we need to do something special here
    if (scheduler->active_item == pkt) {
        LOG("ACTIVE packet remove\n");
    }

    // Use the back-reference to remove from queue, first we try the TX common queue
    int32_t res = PHY_RADIO_SUCCESS;
    if ((res = staticQueueErase(&scheduler->static_queue, &pkt->_phy_queue_item->node)) != STATIC_QUEUE_SUCCESS) {
        // The packet was not in the common queue, check the slot queue
        res = staticQueueErase(&scheduler->slot[pkt->slot].static_queue,
                                    &pkt->_phy_queue_item->node);
    }

    // Notify that the packet send failed
    if (pkt->type != PHY_RADIO_PKT_INTERNAL_SYNC) {
        if ((res = inst->interface->sent_cb(inst->interface, pkt, PHY_RADIO_SEND_FAIL)) != PHY_RADIO_CB_SUCCESS) {
            return res;
        }
    }

    if (res == STATIC_QUEUE_SUCCESS) {
        // Clear the back-reference
        pkt->_phy_queue_item = NULL;
    }

    return res;
}