/**
 * @file:       phy_radio_slot_handler.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation of PHY radio slot handler layer
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

#include "phy_radio_slot_handler.h"
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

#ifndef PHY_RADIO_SLOT_HANDLER_LOG_ENABLE
#define PHY_RADIO_SLOT_HANDLER_LOG_ENABLE (1)
#endif /* PHY_RADIO_SLOT_HANDLER_LOG_ENABLE */

#if PHY_RADIO_SLOT_HANDLER_LOG_ENABLE == 1
#define LOG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG(f_, ...)
#endif /* PHY_RADIO_SLOT_HANDLER_LOG_ENABLE */

#ifdef PHY_RADIO_SLOT_HANDLER_LOG_DEBUG_ENABLE
#define LOG_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_DEBUG(f_, ...)
#endif /* PHY_RADIO_SLOT_HANDLER_LOG_DEBUG_ENABLE */

#ifdef PHY_RADIO_SLOT_HANDLER_LOG_V_DEBUG_ENABLE
#define LOG_V_DEBUG(f_, ...) radio_log((f_), ##__VA_ARGS__)
#else
#define LOG_V_DEBUG(f_, ...)
#endif /* PHY_RADIO_SLOT_HANDLER_LOG_V_DEBUG_ENABLE */

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

typedef enum {
    SLOT_HANDLER_INT_IDLE = 0,
    SLOT_HANDLER_INT_ERROR,
    SLOT_HANDLER_INT_FRAME_GUARD_TIMER,
    SLOT_HANDLER_INT_FRAME_SYNC_SENT,
    SLOT_HANDLER_INT_FRAME_START_TIMER,
    SLOT_HANDLER_INT_SLOT_GUARD_START_TIMER,
    SLOT_HANDLER_INT_SLOT_START_TIMER,
    SLOT_HANDLER_INT_SLOT_END_GUARD_TIMER,
} slotHandlerInterruptEvent_t;

static int32_t manageNewFrameTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index);
static int32_t manageNewFrameStartTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index);
static int32_t manageSlotStartTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index);
static int32_t manageSlotEndGuardTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index);
static int32_t manageSlotGuardTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index);
static int32_t manageStartSyncEvent(phyRadioSlotHandler_t *inst, uint16_t slot_index);

// Start of new frame, after this there is a guard time
static int32_t manageNewFrameTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    // Cancel RX
    inst->current_slot = slot_index;

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    // Inidicate new frame by toggling the GPIO
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Set the interrupt flag for next processing
    inst->timer_interrupt = SLOT_HANDLER_INT_FRAME_GUARD_TIMER;

    // Check what slot we are currently in
    switch(inst->slot[inst->current_slot].current_type) {
        case PHY_RADIO_SLOT_IDLE:
        case PHY_RADIO_SLOT_TX: {
            int32_t res = halRadioCancelReceive(inst->hal_radio_inst, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to cancel %i\n",res);
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX: {
            int32_t res = PHY_RADIO_SLOT_HANDLER_SUCCESS;
            /* TODO I think we can skip this
            if (inst->in_flight) {
                // A packet failed to complete it's transmission, cancel TX mode.
                if ((res = halRadioCancelTransmit(&inst->hal_radio_inst, false)) != HAL_RADIO_SUCCESS) {
                    return res;
                }
                // The management of this will be done in main context
            }
            */

            // Go to RX mode
            inst->hal_interface->pkt_buffer = inst->rx_buffer;
            res = halRadioReceivePackageNB(inst->hal_radio_inst, inst->hal_interface, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to receive %i\n", res);
                return res;
            }
        } break;
        default:
            // Do nothing
            break;
    }

    // The frame start does not require any main context processing
    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

// Start of first slot, time to send sync
static int32_t manageNewFrameStartTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Set the interrupt flag for next processing
    inst->timer_interrupt = SLOT_HANDLER_INT_FRAME_START_TIMER;

    // Check what slot we are currently in
    switch(inst->slot[inst->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
            // Inidicate current state using GPIO
            gpio_put(HAL_RADIO_PIN_TX_RX, 1);
#endif

            // Trigger send of any queued packet
            int32_t res = halRadioQueueSend(inst->hal_radio_inst, false);

            if (res == HAL_RADIO_NOTHING_TO_SEND) {
                // Nothing to do
            } else if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to send %i\n", res);
                return res;
            } else {
                // Packet sent
                inst->timer_interrupt = SLOT_HANDLER_INT_FRAME_SYNC_SENT;
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

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

static int32_t manageSlotGuardTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    // Store the next slot
    inst->current_slot = slot_index;

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(inst->slot[inst->current_slot].current_type) {
        case PHY_RADIO_SLOT_IDLE:
        case PHY_RADIO_SLOT_TX: {

            // If the next slot is TX first Cancel RX
            int32_t res = halRadioCancelReceive(inst->hal_radio_inst, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to cancel %i\n",res);
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX: {

            int32_t res = PHY_RADIO_SLOT_HANDLER_SUCCESS;
            /* TODO I think we can skip this check!
            if (inst->in_flight) {
                // A packet failed to complete it's transmission, cancel TX mode.
                if ((res = halRadioCancelTransmit(&inst->hal_radio_inst, false)) != HAL_RADIO_SUCCESS) {
                    return res;
                }
                // The management of this will be done in main context
            }
            */

            // Go to RX mode
            inst->hal_interface->pkt_buffer = inst->rx_buffer;
            res = halRadioReceivePackageNB(inst->hal_radio_inst, inst->hal_interface, false);

            if (res != HAL_RADIO_SUCCESS) {
                LOG("Failed to receive %i\n", res);
                return res;
            }
        } break;
        default:
            // Do nothing
            break;
    }

    // Trigger main context processing
    inst->timer_interrupt = SLOT_HANDLER_INT_SLOT_GUARD_START_TIMER;

    // The start of a guard period in a new slot does not require any main context processing
    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

static int32_t manageSlotStartTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);

#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
    gpio_put(HAL_RADIO_PIN_TX_RX, 1);
    gpio_put(HAL_RADIO_PIN_TX_RX, 0);
#endif

    // Check what slot we are currently in
    switch(inst->slot[inst->current_slot].current_type) {
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

    // Trigger main context processing
    inst->timer_interrupt = SLOT_HANDLER_INT_SLOT_START_TIMER;

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

static int32_t manageSlotEndGuardTimerInterrupt(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);

    // Check what slot we are currently in
    switch(inst->slot[inst->current_slot].current_type) {
        case PHY_RADIO_SLOT_TX: {
            // Ignore this interrupt
            int32_t res = halRadioCancelTransmit(inst->hal_radio_inst, false);
            if (res != HAL_RADIO_SUCCESS) {
                return res;
            }
        } break;
        case PHY_RADIO_SLOT_RX:

            #ifdef HAL_RADIO_SLOT_GPIO_DEBUG
                gpio_put(HAL_RADIO_PIN_TX_RX, 0);
                gpio_put(HAL_RADIO_PIN_TX_RX, 1);
                gpio_put(HAL_RADIO_PIN_TX_RX, 0);
            #endif
            // Check if radio is busy (mutex locked) - if so, set emergency abort flag
            if (halRadioCheckBusy(inst->hal_radio_inst) == HAL_RADIO_BUSY) {
                // Set emergency abort flag in HAL radio (safe to call from interrupt context)
                halRadioSetRxAbort(inst->hal_radio_inst);
            } else {
                int32_t mode = halRadioGetMode(inst->hal_radio_inst);
                if (mode < HAL_RADIO_SUCCESS) {
                    // Some error occured
                    return mode;
                } else if (mode == HAL_RADIO_RX_ACTIVE) {
                    int32_t res = HAL_RADIO_SUCCESS;
                    if ((res = halRadioCancelReceive(inst->hal_radio_inst, false)) != HAL_RADIO_SUCCESS) {
                        return res;
                    }
                }
            }
            break;
        default:
            // Do nothing
            break;
    }

    // Trigger main context processing
    inst->timer_interrupt = SLOT_HANDLER_INT_SLOT_END_GUARD_TIMER;

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

static int32_t manageStartSyncEvent(phyRadioSlotHandler_t *inst, uint16_t slot_index) {
    UNUSED(slot_index);
    // New sync detected, reset the counters
    inst->current_slot = 0;
    // TODO should we reset everything

    // Set the mandatory peripheral sync RX slot
    int32_t res = phyRadioSlotHandlerSetSlotMain(inst, PHY_RADIO_PERIPHERAL_RX_SLOT, PHY_RADIO_SLOT_RX);
    if (res != PHY_RADIO_SLOT_HANDLER_SUCCESS) {
        return res;
    }

    res = phyRadioSlotHandlerSetSlotCur(inst, PHY_RADIO_PERIPHERAL_RX_SLOT, PHY_RADIO_SLOT_RX);
    if (res != PHY_RADIO_SLOT_HANDLER_SUCCESS) {
        return res;
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerInit(phyRadioSlotHandler_t *inst, const phyRadioSlotHandlerInit_t *init_struct) {
    if (inst == NULL) {
        return PHY_RADIO_SLOT_HANDLER_NULL_ERROR;
    }

    if (init_struct == NULL) {
        return PHY_RADIO_SLOT_HANDLER_NULL_ERROR;
    }

    // Clear the instance
    memset(inst, 0, sizeof(phyRadioSlotHandler_t));

    inst->current_slot = 0;

    inst->phy_radio_inst = init_struct->phy_radio_inst;
    inst->hal_radio_inst = init_struct->hal_radio_inst;
    inst->hal_interface  = init_struct->hal_interface;
    inst->rx_buffer      = init_struct->rx_buffer;

    // Initialize all slots
    for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        // Reset all slots to IDLE
        inst->slot[i].main_type    = PHY_RADIO_SLOT_IDLE;
        inst->slot[i].current_type = PHY_RADIO_SLOT_IDLE;
    }

    LOG_DEBUG("Slot handler initialized\n");

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerDeInit(phyRadioSlotHandler_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_SLOT_HANDLER_NULL_ERROR;
    }

    // Reset module state

    LOG_DEBUG("Slot handler deinitialized\n");

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerProcess(phyRadioSlotHandler_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_SLOT_HANDLER_NULL_ERROR;
    }

    switch (inst->timer_interrupt) {
        case SLOT_HANDLER_INT_IDLE:
            break;
        case SLOT_HANDLER_INT_ERROR:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_ERROR_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_FRAME_GUARD_TIMER:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_NEW_FRAME_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_FRAME_SYNC_SENT:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_NEW_FRAME_SYNC_SENT_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_FRAME_START_TIMER:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_FIRST_SLOT_START_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_SLOT_GUARD_START_TIMER:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_SLOT_GUARD_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_SLOT_START_TIMER:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_SLOT_START_EVENT, inst->current_slot);
        case SLOT_HANDLER_INT_SLOT_END_GUARD_TIMER:
            inst->timer_interrupt = SLOT_HANDLER_INT_IDLE;
            return phyRadioSlotHandlerCallback(inst->phy_radio_inst, SLOT_HANDLER_SLOT_END_GUARD_EVENT, inst->current_slot);
        default:
            break;
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerResetSlots(phyRadioSlotHandler_t *inst) {
    inst->current_slot = 0;

    // Initialize all slots
    for (uint32_t i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        // Reset all slots to IDLE
        inst->slot[i].main_type    = PHY_RADIO_SLOT_IDLE;
        inst->slot[i].current_type = PHY_RADIO_SLOT_IDLE;
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerSetSlotCur(phyRadioSlotHandler_t *inst, uint8_t slot, phyRadioSlotType_t mode) {
    inst->slot[slot].current_type = mode;
    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerSetSlotMain(phyRadioSlotHandler_t *inst, uint8_t slot, phyRadioSlotType_t mode) {
    inst->slot[slot].main_type = mode;

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerRestoreSlots(phyRadioSlotHandler_t *inst) {
    // Loop over all slots and manage their current type configuraion
    for (int i = 0; i < PHY_RADIO_NUM_SLOTS; i++) {
        // If it was a temporary type set it back to main type
        if (inst->slot[i].current_type != inst->slot[i].main_type) {
            inst->slot[i].current_type = inst->slot[i].main_type;
        }
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSlotHandlerGetCurSlotType(phyRadioSlotHandler_t *inst) {
    return inst->slot[inst->current_slot].current_type;
}

int32_t phyRadioSlotHandlerEventManager(phyRadioSlotHandler_t *inst, phyRadioSlotHandlerEvent_t event, uint16_t slot_index) {
    switch (event)
    {
        case SLOT_HANDLER_START_EVENT:
            return manageStartSyncEvent(inst, slot_index);
        case SLOT_HANDLER_NEW_FRAME_EVENT:
        case SLOT_HANDLER_NEW_FRAME_SYNC_SENT_EVENT:
            return manageNewFrameTimerInterrupt(inst, slot_index);
        case SLOT_HANDLER_FIRST_SLOT_START_EVENT:
            return manageNewFrameStartTimerInterrupt(inst, slot_index);
        case SLOT_HANDLER_SLOT_GUARD_EVENT:
            return manageSlotGuardTimerInterrupt(inst, slot_index);
        case SLOT_HANDLER_SLOT_START_EVENT:
            return manageSlotStartTimerInterrupt(inst, slot_index);
        case SLOT_HANDLER_SLOT_END_GUARD_EVENT:
            return manageSlotEndGuardTimerInterrupt(inst, slot_index);
        case SLOT_HANDLER_ERROR_EVENT:
        default:
            LOG("ERROR %i\n", event);
            return PHY_RADIO_SLOT_HANDLER_GEN_ERROR;
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}

int32_t phyRadioSLotHandlerEventInQueue(phyRadioSlotHandler_t *inst) {
    if (inst->timer_interrupt > 0) {
        return PHY_RADIO_SLOT_HANDLER_INTERRUPT_QUEUE;
    }

    return PHY_RADIO_SLOT_HANDLER_SUCCESS;
}