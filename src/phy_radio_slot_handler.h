/**
 * @file:       phy_radio_slot_handler.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for PHY radio slot handler layer
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

#ifndef PHY_RADIO_SLOT_HANDLER_H
#define PHY_RADIO_SLOT_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "phy_radio_common.h"
#include "hal_radio.h"

/**
 * Error codes for slot handler module
 */
typedef enum {
    PHY_RADIO_SLOT_HANDLER_INTERRUPT_QUEUE = 1,
    PHY_RADIO_SLOT_HANDLER_SUCCESS         = 0,
    PHY_RADIO_SLOT_HANDLER_NULL_ERROR      = -23001,
    PHY_RADIO_SLOT_HANDLER_GEN_ERROR       = -23002,
} phyRadioSlotHandlerErr_t;

typedef enum {
    PHY_RADIO_SLOT_IDLE,
    PHY_RADIO_SLOT_RX,
    PHY_RADIO_SLOT_TX,
} phyRadioSlotType_t;

/**
 * Initialization structure for slot handler module
 */
typedef struct {
    // Add initialization parameters here
    phyRadio_t          *phy_radio_inst;
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    cBuffer_t           *rx_buffer;
} phyRadioSlotHandlerInit_t;

/**
 * Main data type for this module
 */
struct phyRadioSlotHandler {
    // Top level instance
    phyRadio_t *phy_radio_inst;

    // Instance parameters
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    cBuffer_t           *rx_buffer;
    uint32_t             timer_interrupt;

    struct {
        // Slot type
        phyRadioSlotType_t current_type;
        phyRadioSlotType_t main_type;
    } slot[PHY_RADIO_NUM_SLOTS];

    // Current slot tracking
    uint8_t  current_slot;
};

/**
 * Initialize the slot handler module
 * Input: phyRadioSlotHandler instance
 * Input: Initialization structure with required parameters
 * Returns: phyRadioSlotHandlerErr_t
 */
int32_t phyRadioSlotHandlerInit(phyRadioSlotHandler_t *inst, const phyRadioSlotHandlerInit_t *init_struct);

/**
 * Deinitialize the slot handler module
 * Input: phyRadioSlotHandler instance
 * Returns: phyRadioSlotHandlerErr_t
 */
int32_t phyRadioSlotHandlerDeInit(phyRadioSlotHandler_t *inst);

/**
 * Process function call
 * Input: phyRadioSlotHandler instance
 * Returns: phyRadioSlotHandlerErr_t
 */
int32_t phyRadioSlotHandlerSetSlotCur(phyRadioSlotHandler_t *inst, uint8_t slot, phyRadioSlotType_t mode);

int32_t phyRadioSlotHandlerSetSlotMain(phyRadioSlotHandler_t *inst, uint8_t slot, phyRadioSlotType_t mode);

/**
 * Process function call
 * Input: phyRadioSlotHandler instance
 * Returns: phyRadioSlotHandlerErr_t
 */
int32_t phyRadioSlotHandlerProcess(phyRadioSlotHandler_t *inst);

int32_t phyRadioSlotHandlerGetCurSlotType(phyRadioSlotHandler_t *inst);

int32_t phyRadioSlotHandlerRestoreSlots(phyRadioSlotHandler_t *inst);

int32_t phyRadioSlotHandlerResetSlots(phyRadioSlotHandler_t *inst);

int32_t phyRadioSlotHandlerEventManager(phyRadioSlotHandler_t *inst, phyRadioSlotHandlerEvent_t event, uint16_t slot_index);

int32_t phyRadioSLotHandlerEventInQueue(phyRadioSlotHandler_t *inst);

#ifdef __cplusplus
}
#endif
#endif /* PHY_RADIO_SLOT_HANDLER_H */
