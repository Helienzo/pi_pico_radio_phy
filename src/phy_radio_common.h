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

// Packet sizes
#define PHY_RADIO_SENDER_ADDR_SIZE    (1)
#define PHY_RADIO_PKT_TYPE_SIZE       (1)
#define PHY_RADIO_OVERHEAD_SIZE       (PHY_RADIO_SENDER_ADDR_SIZE + PHY_RADIO_PKT_TYPE_SIZE)
#define PHY_RADIO_RX_BUFFER_SIZE      (HAL_RADIO_MAX_BUFFER_SIZE + C_BUFFER_ARRAY_OVERHEAD)
#define PHY_RADIO_MAX_PACKET_SIZE     (HAL_RADIO_MAX_PACKET_SIZE - PHY_RADIO_OVERHEAD_SIZE)
#define PHY_RADIO_TOTAL_OVERHEAD_SIZE (PHY_RADIO_OVERHEAD_SIZE + HAL_RADIO_PACKET_OVERHEAD)

// Slot times in us
#ifndef PHY_RADIO_SLOT_TIME_US
#define PHY_RADIO_SLOT_TIME_US (20000)
#endif /* PHY_RADIO_SLOT_TIME_US */

#ifndef PHY_RADIO_SUPERFRAME_LEN
#define PHY_RADIO_SUPERFRAME_LEN (20)
#endif /* PHY_RADIO_SUPERFRAME_LEN */

#ifndef PHY_RADIO_GUARD_TIME_US
#define PHY_RADIO_GUARD_TIME_US (1000)
#endif /* PHY_RADIO_GUARD_TIME_US */

#ifndef PHY_RADIO_BROADCAST_ADDR
#define PHY_RADIO_BROADCAST_ADDR (0xFF)
#endif /* PHY_RADIO_BROADCAST_ADDR */

#define PHY_RADIO_PACKET_TYPE_MASK  0xE0
#define PHY_RADIO_PACKET_TYPE_SHIFT 5

typedef struct phyRadio phyRadio_t;

typedef struct phyRadioPacket phyRadioPacket_t;

typedef enum {
    PHY_RADIO_PKT_DIRECT = 1,        // Direct message heard only by device with matching addr
    PHY_RADIO_PKT_BROADCAST,     // Broadcast packet heard by all
    PHY_RADIO_PKT_INTERNAL_SYNC, // Used internaly by the phy layer to syncronize time between devices
} phyRadioPktType_t;

struct phyRadioPacket {
    cBuffer_t *pkt_buffer;
    uint8_t    type; // Phy packet type, phyRadioPktType_t
    uint8_t    addr; // Destination when sending, Sender when receiving
    uint8_t    slot; // Destination slot when sending, What slot it was received on when receiving
};

typedef enum {
    FRAME_SYNC_START_EVENT,
    FRAME_SYNC_NEW_FRAME_EVENT,
    FRAME_SYNC_GUARD_EVENT,
} phyRadioFrameSyncEvent_t;

int32_t phyRadioFrameSyncCallback(phyRadio_t *inst, phyRadioFrameSyncEvent_t event);

#ifdef __cplusplus
}
#endif
#endif /* PHY_RADIO_COMMON_H */