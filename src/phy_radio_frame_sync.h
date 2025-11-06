/**
 * @file:       phy_radio_frame_sync.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for PHY radio frame synchronization layer
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

#ifndef PHY_RADIO_FRAME_SYNC_H
#define PHY_RADIO_FRAME_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_radio.h"
#include "c_buffer.h"
#include "phy_radio_timer.h"
#include "phy_radio_common.h"

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)	(type *)((char *)(ptr) - offsetof(type,member))
#endif

/* NOTE: The minimum bitrate supported is ~15kbps, due to the maximum representable
 * latency in us is 8191. Bitrates lower than this risk overshooting.
*/

// Sync message and time synchronization
#define PHY_RADIO_SYNC_MSG_MAX_TIME       (8191) // 2^13 - 1
#define PHY_RADIO_SYNC_MSG_TIME_MASK      (0x1FFF)
#define PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT (8)
#define PHY_RADIO_SYNC_MSG_TIME_MSB_MASK  (0x1F)
#define PHY_RADIO_SYNC_MSG_TIME_LSB_MASK  (0xFF)

#ifndef PHY_RADIO_PID_KP
#define PHY_RADIO_PID_KP 0.006f
#endif /* PHY_RADIO_PID_KP */

#ifndef PHY_RADIO_PID_KI
#define PHY_RADIO_PID_KI 0.00001f
#endif /* PHY_RADIO_PID_KI */

#ifndef PHY_RADIO_PID_KD
#define PHY_RADIO_PID_KD 0.06f
#endif /* PHY_RADIO_PID_KD */

typedef enum {
    PHY_RADIO_FRAME_SYNC_SUCCESS          = 0,
    PHY_RADIO_FRAME_SYNC_NULL_ERROR       = -22001,
    PHY_RADIO_FRAME_SYNC_GEN_ERROR        = -22002,
    PHY_RADIO_FRAME_SYNC_SLOT_ERROR       = -22003,
    PHY_RADIO_FRAME_SYNC_MODE_ERROR       = -22004,
    PHY_RADIO_FRAME_SYNC_FRAME_ERROR      = -22005,
    PHY_RADIO_FRAME_SYNC_FRAME_OVERFLOW   = -22015,
} phyRadioFrameSyncErr_t;

// Sync message and time synchronization
#define PHY_RADIO_FRAME_SYNC_TX_TIME_SIZE            (1)
#define PHY_RADIO_FRAME_SYNC_SYNC_MSG_SIZE           (PHY_RADIO_SENDER_ADDR_SIZE + PHY_RADIO_PKT_TYPE_SIZE + PHY_RADIO_FRAME_SYNC_TX_TIME_SIZE + PHY_RADIO_SYNC_GEN_DATA_SIZE)

typedef enum {
    PHY_RADIO_FRAME_SYNC_MODE_TIMER_ERROR = -20092,
    PHY_RADIO_FRAME_SYNC_MODE_HAL_ERROR   = -20091,
    PHY_RADIO_FRAME_SYNC_MODE_IDLE        = 0,
    PHY_RADIO_FRAME_SYNC_MODE_CENTRAL     = 1,
    PHY_RADIO_FRAME_SYNC_MODE_PERIPHERAL  = 2,
    PHY_RADIO_FRAME_SYNC_MODE_SCAN        = 3,
    PHY_RADIO_FRAME_TRANS_TO_PERIPHERAL   = 4,
    PHY_RADIO_FRAME_TRANS_TO_CENTRAL      = 5,
} phyRadioFrameSyncMode_t;

/**
 * Interrupt events that this module can generate
 */
typedef enum {
    PHY_RADIO_FRAME_SYNC_INT_IDLE = 0,
    PHY_RADIO_FRAME_SYNC_INT_NEW_FRAME,
    PHY_RADIO_FRAME_SYNC_INT_FIRST_SLOT,
    PHY_RADIO_FRAME_SYNC_INT_SLOT_GUARD,
    PHY_RADIO_FRAME_SYNC_INT_SLOT_START,
    PHY_RADIO_FRAME_SYNC_INT_SLOT_END_GUARD,
    PHY_RADIO_FRAME_SYNC_INT_ERROR,
} phyRadioFrameSyncInterruptEvent_t;

/**
 * Initialization structure for frame sync module
 */
typedef struct {
    phyRadio_t          *phy_radio_inst;
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    uint8_t              my_address;
    uint8_t              hal_bitrate;
} phyRadioFrameSyncInit_t;

/**
 * Main data type for this module
 */
typedef struct {
    // Top level instance
    phyRadio_t *phy_radio_inst;

    // HalRadio
    halRadio_t          *hal_radio_inst;
    halRadioInterface_t *hal_interface;
    
    // Internal device address
    uint8_t                 my_address;
    phyRadioFrameSyncMode_t mode;
    
    // Sync message and buffers
    uint8_t             sync_message_array[PHY_RADIO_FRAME_SYNC_SYNC_MSG_SIZE + C_BUFFER_ARRAY_OVERHEAD + HAL_RADIO_PACKET_OVERHEAD];
    cBuffer_t           sync_message_buf;
    phyRadioPacket_t    sync_packet;
#if PHY_RADIO_SYNC_GEN_DATA_SIZE > 0
    uint8_t             sync_packet_gen_data[PHY_RADIO_SYNC_GEN_DATA_SIZE]; // Contains custom data configurable by higher layers
    uint8_t             sync_packet_received_gen_data[PHY_RADIO_SYNC_GEN_DATA_SIZE]; // Received custom data
#endif
    
    // Frame management
    phyRadioFrameConfig_t *frame_config; // Pointer to the current frame config
    uint16_t               _frame_ticks[PHY_RADIO_NUM_TICKS_IN_FRAME]; // Frame ticks, used by the timer

    // Timer management
    phyRadioTimer_t        radio_timer;
    phyRadioTimerConfig_t  timer_config;
    volatile uint32_t      timer_interrupt;

    // Frame sync management
    // The actuall time it takes to send a sync message
    uint16_t central_sync_msg_time;

    // The time it took for another central to send it's sync message
    uint64_t pkt_sent_time;

    // The estimated length of the centrals superslot time
    // Due to slight clock drift it might differ from ours
    uint16_t slot_index;
    uint64_t slot_start_time;
    uint64_t frame_duration;
    float    float_frame_duration;

    // PID gains & state
    float   error_prev;
    float   integral;

} phyRadioFrameSync_t;

/**
 * Initialize the frame sync module
 * Input: phyRadioFrameSync instance
 * Input: Initialization structure with required pointers
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncInit(phyRadioFrameSync_t *inst, const phyRadioFrameSyncInit_t *init_struct);

/**
 * Initialize the frame sync module
 * Input: phyRadioFrameSync instance
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncDeInit(phyRadioFrameSync_t *inst);

/**
 * This function should be triggered on each new incomming sync packet
 * Input: phyRadioFrameSync instance
 * Input: The MSB of the phy header
 * Input: The parsed sync packet
 * Input: The raw hal packet
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncNewSync(phyRadioFrameSync_t *inst, uint16_t phy_header_msb, phyRadioPacket_t *phy_packet, halRadioPackage_t* hal_packet);

/**
 * Queue the next sync packet
 * Input: phyRadioFrameSync instance
 * Input: Pointer that is populated with the sync packet
 * Returns: phyRadioFrameSyncErr_t
 */

int32_t phyRadioFrameSyncQueueNextSync(phyRadioFrameSync_t *inst, phyRadioPacket_t **sync_packet);

/**
 * Send the next sync packet
 * Input: phyRadioFrameSync instance
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncSendNextSync(phyRadioFrameSync_t *inst);

/**
 * Set custom data in sync message
 * Input: phyRadioFrameSync instance
 * Input: Pointer to data
 * Input: Size of data in bytes
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncSetCustomData(phyRadioFrameSync_t *inst, uint8_t *data, uint32_t data_size);

/**
 * Clear the custom data in sync message
 * Input: phyRadioFrameSync instance
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncClearCustomData(phyRadioFrameSync_t *inst);

/**
 * Get the latest received custom data
 * Input: phyRadioFrameSync instance
 * Input: Array to populate
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameGetLatestCustomData(phyRadioFrameSync_t *inst, uint8_t **data);

/**
 * Notify that the module that a sync packet has been sent
 * Input: phyRadioFrameSync instance
 * Input: The packet sent
 * Returns: phyRadioFrameSyncErr_t
 */

int32_t phyRadioFrameSyncNotifySyncSent(phyRadioFrameSync_t *inst, halRadioPackage_t* hal_packet);

/**
 * This function is used to manage the current state of the frame sync module
 * Input: phyRadioFrameSync instance
 * Input: New mode
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncSetMode(phyRadioFrameSync_t *inst, phyRadioFrameSyncMode_t mode);
 
/**
 * Check time remaining in frame
 * Input: Frame sync instance
 * Input: Slot
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncTimeLeftInSlot(phyRadioFrameSync_t *inst, uint8_t slot);

/**
 * Process function call
 * Input: phyRadioFrameSync instance
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncProcess(phyRadioFrameSync_t *inst);

/**
 * Configure the frame
 * Input: Frame sync instance
 * Input: Frame configuration
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncSetStructure(phyRadioFrameSync_t *inst, phyRadioFrameConfig_t *frame);

/**
 * Get the current frame config
 * Input: Frame sync instance
 * Input: Frame configuration to populate
 * Returns: phyRadioFrameSyncErr_t
 */
int32_t phyRadioFrameSyncGetStructure(phyRadioFrameSync_t *inst, phyRadioFrameConfig_t **frame);

#ifdef __cplusplus
}
#endif
#endif /* PHY_RADIO_FRAME_SYNC_H */