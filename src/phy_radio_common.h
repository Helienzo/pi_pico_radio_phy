/**
 * @file:       phy_radio_common.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Common structures and definitions shared between PHY radio modules
 * @details     Used to define interfaces between modules
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

#ifndef PHY_RADIO_COMMON_H
#define PHY_RADIO_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "c_buffer.h"
#include "pico/stdlib.h"


#ifndef PHY_RADIO_SYNC_GEN_DATA_SIZE
#define PHY_RADIO_SYNC_GEN_DATA_SIZE (1)
#endif /* PHY_RADIO_SYNC_GEN_DATA_SIZE */

// Packet sizes
#define PHY_RADIO_SENDER_ADDR_SIZE    (1)
#define PHY_RADIO_PKT_TYPE_SIZE       (1)
#define PHY_RADIO_OVERHEAD_SIZE       (PHY_RADIO_SENDER_ADDR_SIZE + PHY_RADIO_PKT_TYPE_SIZE)
#define PHY_RADIO_RX_BUFFER_SIZE      (HAL_RADIO_MAX_BUFFER_SIZE + C_BUFFER_ARRAY_OVERHEAD)
#define PHY_RADIO_MAX_PACKET_SIZE     (HAL_RADIO_MAX_PACKET_SIZE - PHY_RADIO_OVERHEAD_SIZE)
#define PHY_RADIO_TOTAL_OVERHEAD_SIZE (PHY_RADIO_OVERHEAD_SIZE + HAL_RADIO_PACKET_OVERHEAD)

/**
 * TODO what we want to do next is to remake what is defined as a frame and what is defined as a slot.
 * A frame contains multiple slots. There will be no super slot. Instead the frame will be long and contain X number of slots
 * 
 * To make this work we might have to add a separete conversion time for the guard tick, they should have high precision, but a long frame should not
 * 
 */

// Slot times in us
#ifndef PHY_RADIO_SLOT_GUARD_TIME_US
#define PHY_RADIO_SLOT_GUARD_TIME_US (400)
#endif /* PHY_RADIO_SLOT_GUARD_TIME_US */

#ifndef PHY_RADIO_ACTIVE_SYNC_SLOT_TIME_US 
#define PHY_RADIO_ACTIVE_SYNC_SLOT_TIME_US (1500)
#endif /* PHY_RADIO_ACTIVE_SYNC_SLOT_TIME_US */

#ifndef PHY_RADIO_ACTIVE_SLOT_TIME_US 
#define PHY_RADIO_ACTIVE_SLOT_TIME_US (6850)
#endif /* PHY_RADIO_ACTIVE_SLOT_TIME_US */

#ifndef PHY_RADIO_NUM_TICKS_IN_SLOT
#define PHY_RADIO_NUM_TICKS_IN_SLOT (2)
#endif /* PHY_RADIO_NUM_TICKS_IN_SLOT */ 

#ifndef PHY_RADIO_SLOT_TIME_US 
#define PHY_RADIO_SLOT_TIME_US (PHY_RADIO_SLOT_GUARD_TIME_US + PHY_RADIO_ACTIVE_SLOT_TIME_US)
#endif /* PHY_RADIO_SLOT_TIME_US */

// Frame end guard
#ifndef PHY_RADIO_NUM_SLOTS_IN_FRAME
#define PHY_RADIO_NUM_SLOTS_IN_FRAME (4)
#endif /* PHY_RADIO_NUM_SLOTS_IN_FRAME */

#ifndef PHY_RADIO_FRAME_GUARD_US
#define PHY_RADIO_FRAME_GUARD_US (200)
#endif /* PHY_RADIO_FRAME_GUARD_US */

#ifndef PHY_RADIO_FRAME_TIME_US
#define PHY_RADIO_FRAME_TIME_US (PHY_RADIO_NUM_SLOTS_IN_FRAME * PHY_RADIO_SLOT_TIME_US + PHY_RADIO_FRAME_GUARD_US)
#endif /* PHY_RADIO_FRAME_TIME_US */

// Number of guard periods in a slot
#ifndef PHY_RADIO_NUM_TICKS_IN_FRAME
#define PHY_RADIO_NUM_TICKS_IN_FRAME (PHY_RADIO_NUM_SLOTS_IN_FRAME * PHY_RADIO_NUM_TICKS_IN_SLOT)
#endif /* PHY_RADIO_NUM_TICKS_IN_FRAME */

#ifndef PHY_RADIO_BROADCAST_ADDR
#define PHY_RADIO_BROADCAST_ADDR (0xFF)
#endif /* PHY_RADIO_BROADCAST_ADDR */

#define PHY_RADIO_PACKET_TYPE_MASK  0xE0
#define PHY_RADIO_PACKET_TYPE_SHIFT 5

typedef struct phyRadio phyRadio_t;

typedef struct phyRadioPacket   phyRadioPacket_t;
typedef struct phyRadioSlotItem phyRadioSlotItem_t;

typedef enum {
    PHY_RADIO_PKT_DIRECT = 1,    // Direct message heard only by device with matching addr
    PHY_RADIO_PKT_BROADCAST,     // Broadcast packet heard by all
    PHY_RADIO_PKT_INTERNAL_SYNC, // Used internaly by the phy layer to syncronize time between devices
} phyRadioPktType_t;

struct phyRadioPacket {
    cBuffer_t          *pkt_buffer;
    uint8_t             type; // Phy packet type, phyRadioPktType_t
    uint8_t             addr; // Destination when sending, Sender when receiving
    uint8_t             slot; // Destination slot when sending, What slot it was received on when receiving
    phyRadioSlotItem_t *_phy_queue_item; // This is a pointer that helps us keep track of where this packet ends up
};

typedef struct {
    uint16_t slot_start_guard_us; // Guard time at start of slot
    uint16_t slot_length_us;      // Active part of the slot (e.g. TX time)
    uint16_t slot_end_guard_us;   // Guard time at end of slot TODO UNSUPORTED
} phyRadioSlotConfig_t;

typedef struct {
    uint32_t frame_length_us; // Complete time frame

    // Slot allocation for the frame
    phyRadioSlotConfig_t slots[PHY_RADIO_NUM_SLOTS_IN_FRAME];

    uint16_t num_slots; // Total number of slots in the frame

    uint16_t sync_interval; // How often to send the sync

    uint16_t end_guard; // Special guard at the end of the slot
} phyRadioFrameConfig_t;

typedef enum {
    // This is a NOPE event
    FRAME_SYNC_ERROR_EVENT,
    // This event is triggered when a peripheral detects and locks on to a first sync
    FRAME_SYNC_START_EVENT,
    // This event is triggered at the start of a new frame, after this follows a guard period
    FRAME_SYNC_NEW_FRAME_EVENT,
    // This event is triggered at the start of the first slot in the frame
    FRAME_SYNC_FIRST_SLOT_START_EVENT,
    // This event is triggered at the start of a slot, after this follows a guard period
    FRAME_SYNC_SLOT_GUARD_EVENT,
    // This event is triggered at the end of the slot guard, after this the active part of the slot follows
    FRAME_SYNC_SLOT_START_EVENT,
} phyRadioFrameSyncEvent_t;

int32_t phyRadioFrameSyncCallback(phyRadio_t *inst, phyRadioFrameSyncEvent_t event, uint16_t slot_index);

#ifdef __cplusplus
}
#endif
#endif /* PHY_RADIO_COMMON_H */