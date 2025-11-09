/**
 * @file:       phy_radio.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for Phy radio layer, supporting TDMA and ALOHA
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

#ifndef PHY_RADIO_H
#define PHY_RADIO_H

#ifdef __cplusplus
extern "C" {
#endif
#include "hal_radio.h"
#include "static_queue.h"
#include "c_buffer.h"
#include "phy_radio_timer.h"
#include "phy_radio_common.h"
#include "phy_radio_frame_sync.h"

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)	(type *)((char *)(ptr) - offsetof(type,member))
#endif

// TDMA parameters
#ifndef PHY_RADIO_NUM_SLOTS
#define PHY_RADIO_NUM_SLOTS (PHY_RADIO_NUM_SLOTS_IN_FRAME)
#endif /* PHY_RADIO_NUM_SLOTS */

#ifndef PHY_RADIO_NUM_ITEMS
#define PHY_RADIO_NUM_ITEMS (6)
#endif /* PHY_RADIO_NUM_ITEMS */

#ifndef PHY_RADIO_NUM_ITEMS_SLOTS
#define PHY_RADIO_NUM_ITEMS_SLOTS (3)
#endif /* PHY_RADIO_NUM_ITEMS_SLOTS */

#ifndef PHY_RADIO_SYNC_TIMEOUT
#define PHY_RADIO_SYNC_TIMEOUT (3)
#endif /* PHY_RADIO_SYNC_TIMEOUT */

#ifndef PHY_RADIO_PERIPHERAL_RX_SLOT
#define PHY_RADIO_PERIPHERAL_RX_SLOT (0)
#endif /* PHY_RADIO_PERIPHERAL_RX_SLOT */

#ifndef PHY_RADIO_CENTRAL_TX_SLOT
#define PHY_RADIO_CENTRAL_TX_SLOT (0)
#endif /* PHY_RADIO_CENTRAL_TX_SLOT */

// Guard time between two TX packets in the same slot
#ifndef PHY_RADIO_PACKET_GUARD_TIME_US
#define PHY_RADIO_PACKET_GUARD_TIME_US (20)
#endif /* PHY_RADIO_PACKET_GUARD_TIME_US */

// There is a blocking sleep call, this regulates the maximum length that is allowed
#ifndef PHY_RADIO_MAX_BLOCK_DELAY_TIME_US
#define PHY_RADIO_MAX_BLOCK_DELAY_TIME_US (20)
#endif /* PHY_RADIO_MAX_BLOCK_DELAY_TIME_US */

// GPIO pin used to signal current radio mode
#ifdef HAL_RADIO_SLOT_GPIO_DEBUG
#ifndef HAL_RADIO_PIN_TX_RX
#define HAL_RADIO_PIN_TX_RX (22)
#endif /* HAL_RADIO_PIN_TX_RX */
#endif /* HAL_RADIO_SLOT_GPIO_DEBUG */

// Radio configuration
#ifndef PHY_RADIO_DEFAULT_CHANNEL
#define PHY_RADIO_DEFAULT_CHANNEL (868000000)
#endif /* PHY_RADIO_DEFAULT_CHANNEL */

#ifndef PHY_RADIO_BIT_RATE
#define PHY_RADIO_BIT_RATE (HAL_RADIO_BITRATE_300)
#endif /* PHY_RADIO_BIT_RATE */

#ifndef PHY_RADIO_DEFAULT_TX_POWER_DBM
#define PHY_RADIO_DEFAULT_TX_POWER_DBM (0)
#endif /* PHY_RADIO_DEFAULT_TX_POWER_DBM */

typedef enum {
    PHY_RADIO_INTERRUPT_IN_QUEUE = 1,
    PHY_RADIO_SUCCESS            = 0,
    PHY_RADIO_NULL_ERROR    = -20001,
    PHY_RADIO_GEN_ERROR     = -20002,
    PHY_RADIO_DRIVER_ERROR  = -20003,
    PHY_RADIO_GPIO_ERROR    = -20004,
    PHY_RADIO_INVALID_SIZE  = -20005,
    PHY_RADIO_SEND_FAIL     = -20006,
    PHY_RADIO_RECEIVE_FAIL  = -20007,
    PHY_RADIO_BUSY          = -20008,
    PHY_RADIO_BUFFER_ERROR  = -20009,
    PHY_RADIO_TIMER_ERROR   = -20010,
    PHY_RADIO_INVALID_MODE  = -20011,
    PHY_RADIO_INVALID_SLOT  = -20012,
    PHY_RADIO_TDMA_ERROR    = -20013,
    PHY_RADIO_QUEUE_ERROR   = -20014,
} phyRadioErr_t;

typedef enum {
    PHY_RADIO_MODE_FRAME_ERROR = -20093,
    PHY_RADIO_MODE_TIMER_ERROR = -20092,
    PHY_RADIO_MODE_HAL_ERROR   = -20091,
    PHY_RADIO_MODE_IDLE        = 0,
    PHY_RADIO_MODE_SCAN,
    PHY_RADIO_MODE_CENTRAL,
    PHY_RADIO_MODE_PERIPHERAL,
    PHY_RADIO_MODE_ALOHA,
} phyRadioMode_t;

typedef enum {
    PHY_RADIO_SLOT_IDLE,
    PHY_RADIO_SLOT_RX,
    PHY_RADIO_SLOT_TX,
} phyRadioSlotType_t;

typedef enum {
    PHY_RADIO_CB_SET_SCAN       = 2,
    PHY_RADIO_CB_SET_PERIPHERAL = 1, // Notify that the callback completed successfully
    PHY_RADIO_CB_SUCCESS        = 0,        // Notify that the callback completed successfully
    PHY_RADIO_CB_ERROR          = -20050, // Notify that an error occured inside of the callback
    PHY_RADIO_CB_ERROR_INVALID  = -20051, // Notify that the callback was called with invalid params
} phyRadioCbRetVal_t;

typedef enum {
    PHY_RADIO_SYNC_SENT = 1, // On sync sent during central mode
    PHY_RADIO_FIRST_SYNC,    // On first sync when in scan
    PHY_RADIO_RE_SYNC,       // On new sync message during peripheral mode
    PHY_RADIO_CONFLICT_SYNC, // Conflicting sync from other central device
    PHY_RADIO_SYNC_LOST,     // On new sync message during peripheral mode
    PHY_RADIO_FRAME_START, // At the start of a RX slot
    PHY_RADIO_RX_SLOT_START, // At the start of a RX slot
    PHY_RADIO_TX_SLOT_START, // At the start of TX slot (Ater the first package is triggered)
    PHY_RADIO_SCAN_TIMEOUT,  // If a scan timed out with no device found
} phyRadioSyncId_t;

typedef struct phyRadioInterface phyRadioInterface_t;
typedef struct phyRadioSyncState phyRadioSyncState_t;

/**
 * A packet sent callback is triggerd in the following scenarios;
 * 1. When a packet has succefully been sent
 * 2. If an error has occured an no packet was sent
 * 
 * Input: Pointer to the interface containing this callback function pointer
 * Input: Pointer to phy radio packet sent
 * Returns: phyRadioCbRetVal_t or any error code
 *          Any retval that is not SUCCESS will trigger an error.
 */
typedef int32_t (*phyRadioSentCb_t)(phyRadioInterface_t *interface, phyRadioPacket_t *packet, phyRadioErr_t result);

/**
 * A Packet callback is triggered when a new packet has arrived
 * Input: Pointer to the interface containing this callback function pointer
 * Input: Pointer to the incomming phy packet
 * Returns: phyRadioCbRetVal_t or any error code
 *          Any retval that is not SUCCESS will trigger an error.
 */
typedef int32_t (*phyRadioPacketCb_t)(phyRadioInterface_t *interface, phyRadioPacket_t *packet);

/**
 * A sync state callback is triggered:
 *
 * 1. Each time a central has sent a sync message
 * 2. When a phy in scan mode hears the first sync message
 * 3. Each time a peripheral mode phy hears a new sync message
 * 4. If sync has been lost, ie no sync message has been received for some time
 *
 * 5. TODO: At the start of each new slot.
 *    This callback is suitable to use for timekeeping
 *    It is ok to run maintinence tasks during this callback as
 *    nothing important will happen immediately after this call.
 *    (Packets have been send to the radio for processing and no new packets has arrived yet)
 *
 * Input: Pointer to the interface containing this callback function pointer
 * Input: Synd state id, phyRadioSyncId_t
 * Input: Pointer to a populated sync state instance
 * Returns: phyRadioCbRetVal_t or any error code
 *          Any retval that is not SUCCESS will trigger an error.
*/
typedef int32_t (*phyRadioSyncStateCb_t)(phyRadioInterface_t *interface, uint32_t sync_id, const phyRadioSyncState_t *sync_state);

/**
 * Interface used by the phy module to communicate to upstream modules
 */
struct phyRadioInterface {
    phyRadioSentCb_t      sent_cb;
    phyRadioPacketCb_t    packet_cb;
    phyRadioSyncStateCb_t sync_state_cb;
};

// Slot item
struct phyRadioSlotItem {
    phyRadioPacket_t  *pkt;
    staticQueueItem_t node;
};

// Complete scheduler structure
typedef struct {
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    phyRadio_t          *phy_radio_inst;

    // Global TX queue
    phyRadioSlotItem_t items[PHY_RADIO_NUM_ITEMS]; // Circular buffer of slot items
    staticQueue_t      static_queue;

    // Each slot can hold a number of packets
    struct {
        // Slot items and queue
        phyRadioSlotItem_t items[PHY_RADIO_NUM_ITEMS_SLOTS]; // Circular buffer of slot items
        staticQueue_t      static_queue;

        // Slot type
        phyRadioSlotType_t current_type;
        phyRadioSlotType_t main_type;
    } slot[PHY_RADIO_NUM_SLOTS];

    uint8_t            current_slot;
    phyRadioPacket_t   *active_item;
    bool               in_flight;
    uint32_t           scan_timeout_ms;
    uint16_t           packet_delay_time_us; // The time it will take for the receiver to read and decode this packet

    // Superframe management
    uint16_t frame_counter; // Keeping track of number of slots in a superframe
    uint16_t sync_interval; // Keeping track of number of slots in a superframe
    uint16_t sync_counter;  // Keeping track of frames since last sync

    // ForEach context for slot operations
    uint8_t fe_slot_target;

    // Module responsible for syncronizing internal timers
    phyRadioFrameSync_t frame_sync;
} phyRadioTdma_t;


struct phyRadioSyncState {
    uint32_t       sync_slot_number; // The current phy radio slot used for sync messages
    phyRadioMode_t mode;             // The current phy radio mode
    uint8_t        central_address;  // The address to the central device
    uint8_t       *custom_data;      // Custom data sent in the sync message
};

typedef struct {
    phyRadio_t          *phy_radio_inst;
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    uint8_t              my_address;
    uint8_t              hal_bitrate;
} phyRadioTdmaInit_t;


/**
 * The main data type for this module
 */
struct phyRadio {
    // phyRadio internal states
    phyRadioInterface_t *interface;
    phyRadioSyncState_t  sync_state;

    // Internal messages and buffers
    uint8_t              my_address;
    uint8_t              rx_byte_array[PHY_RADIO_RX_BUFFER_SIZE];
    cBuffer_t            rx_buffer;

    // HalRadio
    halRadio_t          hal_radio_inst;
    halRadioInterface_t hal_interface;

    // Timer management
    phyRadioTaskTimer_t      task_timer;
    phyRadioFastTaskTimer_t  fast_task_timer;
    volatile uint8_t         timer_interrupt;

    // Phy TDMA Scheduler
    phyRadioTdma_t     tdma_scheduler;
};

/**
 * Initialize the phy radio module
 * Input: phyRadio instance
 * Input: Upstream interface
 * Returns: phyRadioErr_t
 */
int32_t phyRadioInit(phyRadio_t *inst, phyRadioInterface_t *interface, uint8_t address);

/**
 * DeInitialize the phy radio module
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioDeInit(phyRadio_t *inst);

/**
 * Process the phy radio
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioProcess(phyRadio_t *inst);

/**
 * Check if the phy radio needs to be processed
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioEventInQueue(phyRadio_t *inst);

/**
 * Scan for other phyRadio device
 * Input: phyRadio instance
 * Input: Timeout time in ms, if not device is found a sync_state_callback is called,
 *        If the timeout is set to 0 the scan is indefinate.
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetScanMode(phyRadio_t *inst, uint32_t timeout_ms);

/**
 * Set central mode look for incomming connections
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetCentralMode(phyRadio_t *inst);

/**
 * Allways accept incomming messages and enable send on demand
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetAlohaMode(phyRadio_t *inst);

/**
 * Transition from Central to Peripheral without losing sync
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioTransitionCentralToPeripheral(phyRadio_t *inst, uint8_t new_central_addr);

/**
 * Transition from Peripheral to Central without losing sync
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioTransitionPeripheralToCentral(phyRadio_t *inst);

/**
 * Configure the frame structure used by the phy.
 * Input: phyRadio instance
 * Inpuut: New frame structure
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetFrameStructure(phyRadio_t *inst, phyRadioFrameConfig_t *frame);

/**
 * Send a message on the next available slot in the ongoing frame, or queue for next
 * Input: phyRadio instance
 * Input: phyRadio packet
 * Input: true to send this frame. This can fail if the slot has allready started, or passed, use with care!
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSendOnSlot(phyRadio_t *inst, phyRadioPacket_t* packet, bool this_frame);

/**
 * Receive data on a specific slot. Will automatically switch to TX if send on the same slot
 * Input: phyRadio instance
 * Input: Targeted slot
 * Returns: phyRadioErr_t
 */
int32_t phyRadioReceiveOnSlot(phyRadio_t *inst, uint8_t slot);

/**
 * Set a specific slot as IDLE.
 * Input: phyRadio instance
 * Input: Targeted slot
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetSlotIdle(phyRadio_t *inst, uint8_t slot);

/**
 * Schedule a packet scanning during a slot interval and on a specific channel.
 * Input:  phyRadio instance
 * Input: Start scan slot
 * Input: End scan slot
 * Input: Channel to scan on
 * Returns: phyRadioErr_t
 */
int32_t phyRadioScanDuringSlots(phyRadio_t *inst, uint8_t s_slot, uint8_t e_slot, uint32_t channel);

/**
 * Clear all messages from a slot, make the slot idle, neither RX or TX.
 * Input: phyRadio instance
 * Input: Slot to clear
 * Returns: phyRadioErr_t
 */
int32_t phyRadioClearSlot(phyRadio_t *inst, uint8_t slot);

/**
 * Remove a queued item from a slot
 * Input: phyRadio instance
 * Input: Pointer to the packet to remove (uses pkt->slot to identify the slot)
 * Returns: phyRadioErr_t
 */
int32_t phyRadioRemoveFromSlot(phyRadio_t *inst, phyRadioPacket_t *pkt);

/**
 * Write to custom data field in SYNC message
 * Input: phyRadio instance
 * Input: Pointer to data
 * Input: Num Bytes in data
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSetCustomData(phyRadio_t *inst, uint8_t *data, uint32_t data_size);

/**
 * Clear the custom data field in SYNC message, set to 0x00 for all bytes
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioClearCustomData(phyRadio_t *inst);

/**
 * Get the latest custom data from phy
 * Input: phyRadio instance
 * Input: Data to populate
 * Returns: phyRadioErr_t
 */
int32_t phyRadioGetCustomData(phyRadio_t *inst, uint8_t **data);

#ifdef __cplusplus
}
#endif
#endif /* PHY_RADIO_H */