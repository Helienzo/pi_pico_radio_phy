/**
 * @file:       phy_radio.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation Phy radio layer, supporting TDMA and ALOHA
 *
 * @license: MIT License
 *
 * Copyright (c) 2024 Lucas Wennerholm
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "phy_radio.h"
#include "string.h"
#include "common.h"

#define MODULO_INC(value, base) (((value) + 1) % (base))
#define PHY_RADIO_BROADCAST_ADDR (0xFF)

static int32_t sendDuringSlot(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler, uint8_t slot);
static int32_t queuePopFromSlot(phyRadioTdma_t*     scheduler, uint8_t slot, phyRadioPacket_t **data);
bool repeating_timer_callback(__unused struct repeating_timer *t);
static int64_t prepare_alarm_callback(alarm_id_t id, void *user_data);
static int64_t timer_alarm_callback(alarm_id_t id, void *user_data);
static int32_t cancelAllTimers(phyRadio_t *inst);
static int32_t queuePutFirstInSlot(phyRadioTdma_t* scheduler, uint8_t slot, phyRadioPacket_t* packet);

// This alarm is used to synchronize with a central device
static int64_t timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadio_t *inst =  (phyRadio_t*)user_data;

    // Start the repeating timer, do as little as possible before this point.

    if (!add_repeating_timer_us(inst->tdma_scheduler.slot_duration, repeating_timer_callback, inst, &inst->timer)) {
        inst->sync_state.mode = PHY_RADIO_MODE_TIMER_ERROR;
    }
    inst->timer_active = true;

    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;
    tdma_scheduler->sync_alarm_id = 0; // Reset the alarm

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].type) {
        case PHY_RADIO_SLOT_TX: {
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 1);

            // Trigger send of a queued packet
            int32_t res = halRadioQueueSend(&inst->hal_radio_inst);
            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to queue Send%i\n",res);
                inst->sync_state.mode = PHY_RADIO_MODE_HAL_ERROR;
            }
        } break;
        case PHY_RADIO_SLOT_RX:
            // If this is the case we are allready in RX mode
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 0);
            break;
        default:
            // Do nothing
            break;
    }

    // Schedule the next prepare alarm
    if ((tdma_scheduler->prepare_alarm_id = add_alarm_in_us(PHY_RADIO_TX_PREPARE_US, prepare_alarm_callback, inst, true)) < 0) {
        inst->sync_state.mode = PHY_RADIO_MODE_TIMER_ERROR;
        tdma_scheduler->prepare_alarm_id = 0;
    }

    // Set the interrupt flag for next processing
    inst->timer_interrupt = true;
    return 0;
}

// This alarm tirggers just before the start of a frame to give time for preparing any time critical data
static int64_t prepare_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadio_t *inst =  (phyRadio_t*)user_data;
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;
    tdma_scheduler->prepare_alarm_id = 0;

    // Get the next slot
    tdma_scheduler->current_slot = MODULO_INC(tdma_scheduler->current_slot, PHY_RADIO_NUM_SLOTS);
    tdma_scheduler->superslot_counter = MODULO_INC(tdma_scheduler->superslot_counter, PHY_RADIO_SUPERFRAME_LEN);

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].type) {
        case PHY_RADIO_SLOT_TX: {
            // Cancel RX
            int32_t res = halRadioCancelReceive(&inst->hal_radio_inst);
            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to cancel %i\n",res);
                inst->sync_state.mode = PHY_RADIO_MODE_HAL_ERROR;
            }

            // Check if it is time to send the next sync message, given that we are the central device
            if (tdma_scheduler->superslot_counter == 0) {
                if (inst->sync_state.mode == PHY_RADIO_MODE_CENTRAL) {
                    // Queue package in hal layer
                    cBufferClear(inst->sync_packet.pkt_buffer); // This is safe, lets not check retvals

                    // Create the SYNC packet header
                    uint8_t packet_header[2];
                    uint8_t phy_pkt_type = PHY_RADIO_PKT_INTERNAL_SYNC << PHY_RADIO_PACKET_TYPE_SHIFT;
                    packet_header[0] = (uint8_t)(inst->tdma_scheduler.central_sync_msg_time & PHY_RADIO_SYNC_MSG_TIME_LSB_MASK); // WRITE LSB
                    packet_header[1] = phy_pkt_type | (uint8_t)((inst->tdma_scheduler.central_sync_msg_time >> PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT) &
                                                                 PHY_RADIO_SYNC_MSG_TIME_MSB_MASK); // Write MSB

                    // Add the phy and sync packet header
                    cBufferPrependByte(inst->sync_packet.pkt_buffer, packet_header[0]); // This is safe, lets not check retvals
                    cBufferPrependByte(inst->sync_packet.pkt_buffer, packet_header[1]); // This is safe, lets not check retvals
                    cBufferPrependByte(inst->sync_packet.pkt_buffer, inst->my_address); // This is safe, lets not check retvals

                    // Set the sync packet as active packet
                    inst->hal_interface.pkt_buffer = inst->sync_packet.pkt_buffer;

                    int32_t res = halRadioQueuePackage(&inst->hal_radio_inst,
                                                       &inst->hal_interface,
                                                       inst->sync_packet.addr);

                    inst->tdma_scheduler.active_item = &inst->sync_packet;
                    // A sync packet is now in fligt
                    inst->tdma_scheduler.in_flight = true;

                    if (res != STATIC_QUEUE_SUCCESS) {
                        LOG("Failed to queue %i\n", res);
                        inst->sync_state.mode = PHY_RADIO_MODE_HAL_ERROR;
                    }
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
            res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);
            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to receive %i\n", res);
                inst->sync_state.mode = PHY_RADIO_MODE_HAL_ERROR;
            }

            if (tdma_scheduler->superslot_counter == 0) {
                // Keep track of number of syncs
                inst->tdma_scheduler.sync_counter++;
            }
        } break;
        default:
            // Do nothing
            break;
    }

    return 0;
}

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    // Get the current phy radio instance
    phyRadio_t *inst =  (phyRadio_t*)t->user_data;
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // Get the current time
    tdma_scheduler->slot_start_time = time_us_64();

    // Check what slot we are currently in
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].type) {
        case PHY_RADIO_SLOT_TX: {
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 1);

            // Store the absolute time when this packet tx started
            inst->tdma_scheduler.pkt_sent_time = time_us_64();

            // Trigger send of a queued packet
            int32_t res = halRadioQueueSend(&inst->hal_radio_inst);
            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to queue send %i\n", res);
                inst->sync_state.mode = PHY_RADIO_MODE_HAL_ERROR;
            }
        } break;
        case PHY_RADIO_SLOT_RX:
            // If this is the case we are allready in RX mode
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 0);
            break;
        default:
            // Do nothing
            break;
    }

    // Schedule the next prepare alarm
    if ((tdma_scheduler->prepare_alarm_id = add_alarm_in_us(PHY_RADIO_TX_PREPARE_US, prepare_alarm_callback, inst, true)) < 0) {
        inst->sync_state.mode = PHY_RADIO_MODE_TIMER_ERROR;
        tdma_scheduler->prepare_alarm_id = 0;
    }

    // Set the interrupt flag for next processing
    inst->timer_interrupt = true;

    return true;
}

static int32_t sendDuringCb(phyRadio_t *inst, phyRadioTdma_t* tdma_scheduler, uint8_t slot) {
    // TODO here we should check if there is another message in the slot queue, and if it fits in the time left send it
    // if it does not leave it in the queue
    int32_t res = queuePopFromSlot(tdma_scheduler, tdma_scheduler->current_slot, &tdma_scheduler->active_item);
    if (res != STATIC_QUEUE_SUCCESS) {
        if (res == STATIC_QUEUE_EMPTY) {
            // Go to default mode
            return halRadioCancelTransmit(&inst->hal_radio_inst); // Be in standby during empty TX slots
        }
        return PHY_RADIO_TDMA_ERROR;
    }

    LOG_DEBUG("Send during CB\n");

    phyRadioPacket_t* active_packet = tdma_scheduler->active_item;

    // Assign the current active buffer to the interface
    inst->hal_interface.pkt_buffer = active_packet->pkt_buffer;

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
    UNUSED(result);
    phyRadio_t * inst = CONTAINER_OF(interface, phyRadio_t, hal_interface);

    // Check if we have any active packet in flight, inform higher layer that it has been sent
    int32_t cb_res = PHY_RADIO_CB_SUCCESS;
    if (inst->tdma_scheduler.in_flight && (inst->tdma_scheduler.active_item->type != PHY_RADIO_PKT_INTERNAL_SYNC)) {
        cb_res = inst->interface->sent_cb(inst->interface, inst->tdma_scheduler.active_item, PHY_RADIO_SUCCESS);
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
                inst->tdma_scheduler.pkt_sent_time = hal_packet->time - inst->tdma_scheduler.pkt_sent_time;
                // If this calculation was correct the send time should never be more than 2ms
                // This protects from errounous behaviour, could perhaps be removed
                if (inst->tdma_scheduler.pkt_sent_time < PHY_RADIO_SYNC_MSG_MAX_TIME) {
                    inst->tdma_scheduler.central_sync_msg_time = (uint16_t)inst->tdma_scheduler.pkt_sent_time;
                }
            }

            // Fall through, for now we just return to RX mode when in central mode
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
            int32_t res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);
            // BUSY is ok, it most likely means a new package has been queued
            if (res != HAL_RADIO_SUCCESS && res != HAL_RADIO_BUSY) {
                LOG("Return to RX failed %i\n", res);
                return res;
            }
            // Indicate to hal not to change any internal state
            return HAL_RADIO_CB_DO_NOTHING;
        } break;
        default:
            break;
    }

    return HAL_RADIO_CB_SUCCESS;
}

static int32_t syncWithCentral(phyRadio_t *inst, uint64_t toa, uint16_t sync_time) {
    phyRadioTdma_t *tdma_scheduler = &inst->tdma_scheduler;

    // New superslot detected, new sync detected
    tdma_scheduler->superslot_counter = 0;
    tdma_scheduler->sync_counter = 0;

    if (inst->sync_state.mode == PHY_RADIO_MODE_PERIPHERAL) {
        // Resync the repeating timer
        if (!cancel_repeating_timer(&inst->timer)) {
            return PHY_RADIO_TIMER_ERROR;
        }
        inst->timer_active = false;

        // Compute when the next slot should start
        uint64_t next_slot_start = inst->tdma_scheduler.slot_duration - sync_time;

        if ((tdma_scheduler->sync_alarm_id = add_alarm_in_us(next_slot_start, timer_alarm_callback, inst, true)) < 0) {
            return PHY_RADIO_TIMER_ERROR;
        }

        // Cancel any active prepare alarm
        if (tdma_scheduler->prepare_alarm_id != 0) {
            if (!cancel_alarm(tdma_scheduler->prepare_alarm_id)) {
                return PHY_RADIO_TIMER_ERROR;
            }
        }

        // Set the next prepare to trigger 1ms before the slot timer
        next_slot_start -= PHY_RADIO_GUARD_TIME_US;

        if ((tdma_scheduler->prepare_alarm_id = add_alarm_in_us(next_slot_start, prepare_alarm_callback, inst, true)) < 0) {
            tdma_scheduler->prepare_alarm_id = 0;
            return PHY_RADIO_TIMER_ERROR;
        }

        // Calculate the offset between our superslot and the central super slot
        int64_t slot_diff = (int64_t)toa - (int64_t)tdma_scheduler->slot_start_time;
        if (slot_diff < PHY_RADIO_SLOT_TIME_US && slot_diff > -PHY_RADIO_SLOT_TIME_US) {
            float offset = (float)(slot_diff);
            // Exponential moving average of the difference
            inst->tdma_scheduler.offset_estimate = inst->tdma_scheduler.offset_estimate*0.9 + offset*0.1;
            float f_sync_time = (float)(sync_time);

            // Compute the error, per slot
            float err = (inst->tdma_scheduler.offset_estimate - f_sync_time) * inst->tdma_scheduler.inverse_of_num_slots;

            // Compute the new slot duration
            inst->tdma_scheduler.float_slot_duration = inst->tdma_scheduler.float_slot_duration*0.90 + (inst->tdma_scheduler.float_slot_duration + err)*0.1;
            inst->tdma_scheduler.slot_duration = (int32_t)(inst->tdma_scheduler.float_slot_duration);

            LOG_DEBUG("Sync time %u, offset %i, diff %i\n", sync_time, (int32_t)inst->tdma_scheduler.offset_estimate, (int32_t)slot_diff);
            LOG("Duriation Error: %i us\n", (int32_t)err);
            LOG("SLOT DURATION:   %i us\n", (int32_t)inst->tdma_scheduler.slot_duration);
        }
    } else {
        // New sync detected, reset the counters
        tdma_scheduler->current_slot = 0;

        // Compute when the next slot should start
        uint64_t next_slot_start = PHY_RADIO_SLOT_TIME_US - sync_time;

        if ((tdma_scheduler->sync_alarm_id = add_alarm_in_us(next_slot_start, timer_alarm_callback, inst, true)) < 0) {
            return PHY_RADIO_TIMER_ERROR;
        }

        // Set the next prepare to trigger 1ms before the slot timer
        next_slot_start -= PHY_RADIO_GUARD_TIME_US;

        if ((tdma_scheduler->prepare_alarm_id = add_alarm_in_us(next_slot_start, prepare_alarm_callback, inst, true)) < 0) {
            tdma_scheduler->prepare_alarm_id = 0;
            return PHY_RADIO_TIMER_ERROR;
        }

        // Initialize the offset time
        float f_sync_time = (float)sync_time;
        inst->tdma_scheduler.offset_estimate = f_sync_time;

        // Initialize all queues, TODO fix the assignment
        for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
            // Set slot type based on it's index
            if (i % 2 == 0) {
                // All even slots are of RX type (Including slot 0)
                inst->tdma_scheduler.slot[i].type = PHY_RADIO_SLOT_RX;
            } else {
                // All uneven slots are of TX type (Including slot 0)
                inst->tdma_scheduler.slot[i].type = PHY_RADIO_SLOT_TX;
            }
        }
    }

    return PHY_RADIO_SUCCESS;
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
    uint16_t phy_header_msb = cBufferReadByte(interface->pkt_buffer);

    // Mask out the packet type
    uint8_t phy_pkt_type = (phy_header_msb & PHY_RADIO_PACKET_TYPE_MASK) >> PHY_RADIO_PACKET_TYPE_SHIFT;

    phyRadioPacket_t new_packet = {
        .pkt_buffer = interface->pkt_buffer,
        .type       = phy_pkt_type,
        .addr       = sender_address,
        .slot       = inst->tdma_scheduler.current_slot,
    };

    // Manage what happens next
    switch(inst->sync_state.mode) {
        case PHY_RADIO_MODE_SCAN:
            // Check if the incomming packet is a broadcast, sync packet
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR && phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC) {

                // Double check that the length of the packet is correct
                if (bytes_in_packets < PHY_RADIO_SYNC_MSG_SIZE) {
                    return HAL_RADIO_CB_DO_NOTHING;
                }

                // Parse the sync message time
                uint16_t sync_msg_time = cBufferReadByte(interface->pkt_buffer);
                phy_header_msb <<= PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT;
                sync_msg_time |= phy_header_msb;
                sync_msg_time &= PHY_RADIO_SYNC_MSG_TIME_MASK; // Mask out the relevant bits

                // Central device has been found, populate the slots with correct values
                int32_t res = syncWithCentral(inst, hal_packet->time, sync_msg_time);
                if (res != PHY_RADIO_SUCCESS) {
                    LOG("SYNC FAILED! %i\n", res);
                    return res;
                }

                // Switch the phy to peripheral mode, to enable the higher layer to send messages
                inst->sync_state.mode = PHY_RADIO_MODE_PERIPHERAL;

                // Notify that a device has been found, TODO fix assignment
                inst->sync_state.tx_slot_number  = PHY_RADIO_PERIPHERAL_TX_SLOT;
                inst->sync_state.central_address = sender_address;
                int32_t cb_res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_FIRST_SYNC, &inst->sync_state);

                // Manage the callback result
                switch (cb_res) {
                    case PHY_RADIO_CB_SUCCESS:
                        break;
                    case PHY_RADIO_CB_SET_PERIPHERAL:
                        break;
                    case PHY_RADIO_CB_SET_SCAN: {
                        // Reset the sync state, TODO fix assignment
                        inst->sync_state.tx_slot_number = PHY_RADIO_PERIPHERAL_TX_SLOT;
                        inst->sync_state.central_address = 0x00;
                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Return to scan mode
                        if ((res = phyRadioSetScanMode(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error %i!\n", cb_res);
                        return cb_res;
                }
            }
            // Make sure we stay in RX even if we got something which is not a sync
            return HAL_RADIO_CB_DO_NOTHING;
        case PHY_RADIO_MODE_PERIPHERAL:
            // Check if the incomming packet is a new sync from the current central
            // We can only be synced to one central at a time
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR &&
                phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC &&
                inst->sync_state.central_address == sender_address) {

                // Double check that the length of the packet is correct
                if (bytes_in_packets < PHY_RADIO_SYNC_MSG_SIZE) {
                    return HAL_RADIO_CB_DO_NOTHING;
                }

                // Parse the sync message time
                uint16_t sync_msg_time = cBufferReadByte(interface->pkt_buffer);
                phy_header_msb <<= PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT;
                sync_msg_time |= phy_header_msb;
                sync_msg_time &= PHY_RADIO_SYNC_MSG_TIME_MASK; // Mask out the relevant bits

                // Resync with central 
                int32_t res = syncWithCentral(inst, hal_packet->time, sync_msg_time);
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
                        // Reset the sync state, TODO fix assignment
                        inst->sync_state.tx_slot_number = PHY_RADIO_PERIPHERAL_TX_SLOT;
                        inst->sync_state.central_address = 0x00;
                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Go to scan mode
                        if ((res = phyRadioSetScanMode(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error %i!\n", cb_res);
                        return cb_res;
                }

                // Notify that the radio should stay in RX mode
                return HAL_RADIO_CB_DO_NOTHING;
            }
            // Fall through
        case PHY_RADIO_MODE_CENTRAL:
            // Check if it is a sync packet from another central
            if (hal_packet->address == PHY_RADIO_BROADCAST_ADDR && phy_pkt_type == PHY_RADIO_PKT_INTERNAL_SYNC) {

                // Double check that the length of the packet is correct
                if (bytes_in_packets < PHY_RADIO_SYNC_MSG_SIZE) {
                    return HAL_RADIO_CB_DO_NOTHING;
                }

                // Parse the sync message time
                uint16_t sync_msg_time = cBufferReadByte(interface->pkt_buffer);
                phy_header_msb <<= PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT;
                sync_msg_time |= phy_header_msb;
                sync_msg_time &= PHY_RADIO_SYNC_MSG_TIME_MASK; // Mask out the relevant bits

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
                        int32_t res = syncWithCentral(inst, hal_packet->time, sync_msg_time);
                        if (res != PHY_RADIO_SUCCESS) {
                            LOG("SYNC FAILED! %i\n", res);
                            return res;
                        }

                        // Notify that a device has been found, TODO fix assignment
                        inst->sync_state.tx_slot_number  = PHY_RADIO_PERIPHERAL_TX_SLOT;
                        inst->sync_state.central_address = sender_address;

                        // Set peripheral mode
                        inst->sync_state.mode = PHY_RADIO_MODE_PERIPHERAL;
                    } break;
                    case PHY_RADIO_CB_SET_SCAN: {
                        // Reset the sync state, TODO fix assignment
                        inst->sync_state.tx_slot_number = PHY_RADIO_PERIPHERAL_TX_SLOT;
                        inst->sync_state.central_address = 0x00;
                        // Cancel all active timers
                        if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }

                        // Go to scan mode
                        if ((res = phyRadioSetScanMode(inst)) != PHY_RADIO_SUCCESS) {
                            return res;
                        }
                    } break;
                    default:
                        LOG("PHY CB Error %i!\n", cb_res);
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
                LOG("PHY CB Error %i!\n", cb_res);
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
    }

    return result;
}

static int32_t queueJustPopFromSlot(phyRadioTdma_t* scheduler,
                                    uint8_t             slot)
{
    // Get the next item from the queue and remove it
    staticQueueItem_t* item;
    int32_t            result = staticQueuePop(&scheduler->slot[slot].static_queue, &item);
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

static void queueClearSlot(phyRadioTdma_t* scheduler,
                           uint8_t         slot)
{
    staticQueueClear(&scheduler->slot[slot].static_queue);
}

static int32_t initTdmaScheduler(phyRadioTdma_t *inst) {
    // Reset all counters
    inst->current_slot = 0;
    inst->superslot_counter = 0;
    inst->sync_counter = 0;
    inst->in_flight    = false;

    // Reset the alarm ID's
    inst->prepare_alarm_id = 0;
    inst->sync_alarm_id = 0;

    // Reset all time keepers
    inst->central_sync_msg_time = 0;
    inst->pkt_sent_time         = 0;
    inst->slot_start_time       = 0;
    inst->slot_duration         = PHY_RADIO_SLOT_TIME_US;
    inst->float_slot_duration   = (float)PHY_RADIO_SLOT_TIME_US;
    inst->offset_estimate       = 0.0;
    inst->inverse_of_num_slots = 1.0/((float)PHY_RADIO_SUPERFRAME_LEN);

    // Initialize all queues
    for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        int32_t res = STATIC_QUEUE_INIT(&inst->slot[i].static_queue, inst->slot[i].items, PHY_RADIO_NUM_ITEMS_SLOTS);
        if (res != STATIC_QUEUE_SUCCESS) {
            return res;
        }

        // Set slot type based on it's index, TODO fix the assignment
        if (i % 2 == 0) {
            // All even slots are of RX type (Including slot 0)
            inst->slot[i].type = PHY_RADIO_SLOT_TX;
        } else {
            // All uneven slots are of TX type (Including slot 0)
            inst->slot[i].type = PHY_RADIO_SLOT_RX;
        }
    }

    return PHY_RADIO_SUCCESS;
};

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

    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].type) {
        case PHY_RADIO_SLOT_TX: {
            // Notify that sync has been sent
            if (tdma_scheduler->superslot_counter == 0) {
                // Notify that sync has been sent, TODO fix assignment
                inst->sync_state.tx_slot_number = PHY_RADIO_CENTRAL_TX_SLOT;
                inst->sync_state.central_address = inst->my_address;
                res = inst->interface->sync_state_cb(inst->interface, PHY_RADIO_SYNC_SENT, &inst->sync_state);

                // If the callback returns bad values stop and return errors
                if (res < PHY_RADIO_CB_SUCCESS) {
                    LOG("PHY CB Error %i!\n", res);
                    return res;
                }
            }

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
    UNUSED(inst);
    return PHY_RADIO_SUCCESS;
}

static int32_t cancelAllTimers(phyRadio_t *inst) {
    phyRadioTdma_t* tdma_scheduler = &inst->tdma_scheduler;

    // Cancel active timer
    if (inst->timer_active) {
        if (!cancel_repeating_timer(&inst->timer)) {
            return PHY_RADIO_TIMER_ERROR;
        }
        inst->timer_active = false;
    }

    // Cancel any active alarm
    if (tdma_scheduler->prepare_alarm_id != 0) {
        if (!cancel_alarm(tdma_scheduler->prepare_alarm_id)) {
            return PHY_RADIO_TIMER_ERROR;
        }
        tdma_scheduler->prepare_alarm_id = 0;
    }

    if (tdma_scheduler->sync_alarm_id != 0) {
        if (!cancel_alarm(tdma_scheduler->sync_alarm_id)) {
            return PHY_RADIO_TIMER_ERROR;
        }
        tdma_scheduler->sync_alarm_id = 0;
    }

    return PHY_RADIO_SUCCESS;
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
                // Reset the sync state, TODO fix assignment
                inst->sync_state.tx_slot_number = PHY_RADIO_PERIPHERAL_TX_SLOT;
                inst->sync_state.central_address = 0x00;
                inst->tdma_scheduler.slot_duration = PHY_RADIO_SLOT_TIME_US;
                inst->tdma_scheduler.float_slot_duration = (float)PHY_RADIO_SLOT_TIME_US;
                // Cancel all active timers
                if ((result = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
                    return result;
                }

                // Go to scan mode
                if ((result = phyRadioSetScanMode(inst)) != PHY_RADIO_SUCCESS) {
                    return result;
                }
            } break;
            case PHY_RADIO_CB_SET_PERIPHERAL:
                // Stay in peripheral mode
                break;
            case PHY_RADIO_CB_SUCCESS:
                break;
            default:
                LOG("PHY CB Error %i!\n", cb_res);
                return cb_res;
        }

        return PHY_RADIO_SUCCESS;
    }

    int32_t res = PHY_RADIO_SUCCESS;
    switch(tdma_scheduler->slot[tdma_scheduler->current_slot].type) {
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

int32_t phyRadioProcess(phyRadio_t *inst) {
    if (inst->timer_interrupt) {
        // Manage timer tasks
        inst->timer_interrupt = false;
        LOG_DEBUG("Repeat at %lld\n", time_us_64());

        switch(inst->sync_state.mode) {
            case PHY_RADIO_MODE_CENTRAL:
                return processCentral(inst);
            case PHY_RADIO_MODE_PERIPHERAL:
                return processPeripheral(inst);
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
    }

    int32_t res = halRadioProcess(&inst->hal_radio_inst);

    switch (res) {
        case HAL_RADIO_RECEIVE_FAIL:
            // This indicates that a bad interrupt occured
            LOG("Hal Radio Receive failed\n");
            // Just restart radio reception
            res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);
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
    };

    int32_t res = halRadioInit(&inst->hal_radio_inst, hal_config);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->hal_interface.package_cb  = halRadioPackageCb;
    inst->hal_interface.pkg_sent_cb = halRadioSentCb;

    inst->interface  = interface;
    inst->my_address = address;

    res = initTdmaScheduler(&inst->tdma_scheduler);

    // Calculate a best effort sync message time estimate to initialize this value
    inst->tdma_scheduler.central_sync_msg_time = halRadioBitRateToDelayUs(&inst->hal_radio_inst, hal_config.bitrate, PHY_RADIO_SYNC_MSG_SIZE);

    // Init the TX RX GPIO
    gpio_init(HAL_RADIO_PIN_TX_RX);
    gpio_set_dir(HAL_RADIO_PIN_TX_RX, GPIO_OUT);

    // Init the sync package
    if(cBufferInit(&inst->sync_message_buf, inst->sync_message_array, PHY_RADIO_SYNC_MSG_SIZE + 1 + HAL_RADIO_PACKET_OVERHEAD) != C_BUFFER_SUCCESS) {
        return PHY_RADIO_GEN_ERROR;
    }

    inst->sync_packet.addr = PHY_RADIO_BROADCAST_ADDR;
    inst->sync_packet.type = PHY_RADIO_PKT_INTERNAL_SYNC;
    inst->sync_packet.slot = 0;
    inst->sync_packet.pkt_buffer = &inst->sync_message_buf;

    // Init the RX buffer
    if(cBufferInit(&inst->rx_buffer, inst->rx_byte_array, PHY_RADIO_RX_BUFFER_SIZE) != C_BUFFER_SUCCESS) {
        return PHY_RADIO_GEN_ERROR;
    }

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetScanMode(phyRadio_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    int32_t res = PHY_RADIO_SUCCESS;
    // Cancel any active timers
    if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    // Scan for the broadcast address
    inst->hal_interface.pkt_buffer = &inst->rx_buffer;
    res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_SCAN;

    // Reset all counters
    inst->tdma_scheduler.current_slot = 0;
    inst->tdma_scheduler.superslot_counter = 0;
    inst->tdma_scheduler.sync_counter = 0;

    return PHY_RADIO_SUCCESS;
}

int32_t phyRadioSetCentralMode(phyRadio_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_NULL_ERROR;
    }

    // Cancel any active timers
    int32_t res = PHY_RADIO_SUCCESS;
    if ((res = cancelAllTimers(inst)) != PHY_RADIO_SUCCESS) {
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_CENTRAL;
    // Start broadcasting a sync message to enable other units to adjust their clocks

    if (!add_repeating_timer_us(PHY_RADIO_SLOT_TIME_US, repeating_timer_callback, inst, &inst->timer)) {
        return PHY_RADIO_TIMER_ERROR;
    }
    inst->timer_active = true;

    // Reset all counters
    inst->tdma_scheduler.current_slot = 0;
    inst->tdma_scheduler.superslot_counter = 0;
    inst->tdma_scheduler.sync_counter = 0;

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
    res = halRadioReceivePackageNB(&inst->hal_radio_inst, &inst->hal_interface);

    if (res != HAL_RADIO_SUCCESS) {
        return res;
    }

    inst->sync_state.mode = PHY_RADIO_MODE_ALOHA;

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

    return PHY_RADIO_SUCCESS;
}

static inline int32_t sendCentral(phyRadio_t *inst, phyRadioPacket_t* packet) {
    // Check that the slot is valid, protects agains oob
    if (packet->slot >= PHY_RADIO_NUM_SLOTS) {
        return PHY_RADIO_INVALID_SLOT;
    }

    // Make sure that it is valid to send on the target slot
    if (inst->tdma_scheduler.slot[packet->slot].type != PHY_RADIO_SLOT_TX) {
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
        packet->addr = RFM69_DEFAULT_BROADCAST_ADDR;
    }

    // Store the packet in the outgoing slot
    res = queuePutInSlot(&inst->tdma_scheduler, packet->slot, packet);
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