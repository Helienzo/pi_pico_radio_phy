/**
 * @file:       phy_radio.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for Phy radio layer, supporting TDMA and ALOHA
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

#ifndef PHY_RADIO_H
#define PHY_RADIO_H
#include "pico/stdlib.h"
#include "hal_radio.h"
#include "static_queue.h"
#include "pico/time.h"
#include "c_buffer.h"

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)	(type *)((char *)(ptr) - offsetof(type,member))
#endif

// Packet sizes
#define PHY_RADIO_SENDER_ADDR_SIZE 1
#define PHY_RADIO_PKT_TYPE_SIZE    1
#define PHY_RADIO_OVERHEAD_SIZE       (PHY_RADIO_SENDER_ADDR_SIZE + PHY_RADIO_PKT_TYPE_SIZE)
#define PHY_RADIO_RX_BUFFER_SIZE      (HAL_RADIO_MAX_BUFFER_SIZE + C_BUFFER_ARRAY_OVERHEAD)
#define PHY_RADIO_MAX_PACKET_SIZE     (HAL_RADIO_MAX_PACKET_SIZE - PHY_RADIO_OVERHEAD_SIZE)
#define PHY_RADIO_TOTAL_OVERHEAD_SIZE (PHY_RADIO_OVERHEAD_SIZE + HAL_RADIO_PACKET_OVERHEAD)

/* NOTE: The minimum bitrate supported is ~15kbps, due to the maximum representable
 * latency in us is 8191. Bitrates lower than this risk overshooting.
*/
// Sync message and time synchronization
#define PHY_RADIO_TX_TIME_SIZE            1
#define PHY_RADIO_SYNC_MSG_SIZE           (PHY_RADIO_SENDER_ADDR_SIZE + PHY_RADIO_PKT_TYPE_SIZE + PHY_RADIO_TX_TIME_SIZE)
#define PHY_RADIO_SYNC_MSG_MAX_TIME       8191 // 2^13 - 1
#define PHY_RADIO_SYNC_MSG_TIME_MASK      0x1FFF
#define PHY_RADIO_SYNC_MSG_TIME_MSB_SHIFT 8
#define PHY_RADIO_SYNC_MSG_TIME_MSB_MASK  0x1F
#define PHY_RADIO_SYNC_MSG_TIME_LSB_MASK  0xFF

// TDMA parameters
#define PHY_RADIO_NUM_SLOTS          2
#define PHY_RADIO_NUM_ITEMS_SLOTS    3
#define PHY_RADIO_SUPERFRAME_LEN     20
#define PHY_RADIO_SYNC_TIMEOUT       3
#define PHY_RADIO_PERIPHERAL_TX_SLOT 1
#define PHY_RADIO_CENTRAL_TX_SLOT    0

// Slot times in us
#define PHY_RADIO_SLOT_TIME_US  (20000)
#define PHY_RADIO_GUARD_TIME_US (1000)
#define PHY_RADIO_SUPERFRAME_TIME_US (PHY_RADIO_SLOT_TIME_US * PHY_RADIO_SUPERFRAME_LEN)
#define PHY_RADIO_SUPERFRAME_TIME_MS (PHY_RADIO_SUPERFRAME_TIME_US/1000)

// Guard time before the next slot start enables preparing the radio for the next slot
#define PHY_RADIO_TX_PREPARE_US (PHY_RADIO_SLOT_TIME_US - PHY_RADIO_GUARD_TIME_US)

// GPIO pin used to signal current radio mode
#define HAL_RADIO_PIN_TX_RX (22)

// Radio configuration
#define PHY_RADIO_DEFAULT_CHANNEL      (868)
#define PHY_RADIO_BIT_RATE             (HAL_RADIO_BITRATE_150)
#define PHY_RADIO_DEFAULT_TX_POWER_DBM (0)
#define PHY_RADIO_BROADCAST_ADDR       (0xFF)

typedef enum {
    PHY_RADIO_SUCCESS,
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
    PHY_RADIO_MODE_TIMER_ERROR = -20092,
    PHY_RADIO_MODE_HAL_ERROR   = -20091,
    PHY_RADIO_MODE_IDLE,
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

#define PHY_RADIO_PACKET_TYPE_MASK  0xE0
#define PHY_RADIO_PACKET_TYPE_SHIFT 5

typedef enum {
    PHY_RADIO_PKT_DIRECT = 1,        // Direct message heard only by device with matching addr
    PHY_RADIO_PKT_BROADCAST,     // Broadcast packet heard by all
    PHY_RADIO_PKT_INTERNAL_SYNC, // Used internaly by the phy layer to syncronize time between devices
} phyRadioPktType_t;

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
    PHY_RADIO_RX_SLOT_START, // At the start of a RX slot
    PHY_RADIO_TX_SLOT_START, // At the start of TX slot (Ater the first package is triggered)
    PHY_RADIO_SCAN_TIMEOUT,  // If a scan timed out with no device found
} phyRadioSyncId_t;

typedef struct phyRadioInterface phyRadioInterface_t;
typedef struct phyRadioPacket    phyRadioPacket_t;
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
typedef struct {
    phyRadioPacket_t  *pkt;
    staticQueueItem_t node;
} phyRadioSlotItem_t;

// Complete scheduler structure
typedef struct {
    // Each slot can hold a number of packets
    struct {
        // Slot items and queue
        phyRadioSlotItem_t items[PHY_RADIO_NUM_ITEMS_SLOTS]; // Circular buffer of slot items
        staticQueue_t      static_queue;

        // Slot type
        phyRadioSlotType_t type;
    } slot[PHY_RADIO_NUM_SLOTS];

    uint8_t            current_slot;
    phyRadioPacket_t   *active_item;
    bool               in_flight;
    alarm_id_t         prepare_alarm_id;
    alarm_id_t         sync_alarm_id;
    uint64_t           pkt_sent_time;
    uint32_t           scan_timeout_ms;

    // Superframe management
    uint32_t superslot_counter; // Keeping track of number of slots in a superframe
    uint32_t sync_counter;      // Keeping track of frames since last sync

    // The actuall time it takes to send a sync message
    uint16_t central_sync_msg_time;

    // The estimated length of the centrals superslot time
    // Due to slight clock drift it might differ from ours
    uint64_t slot_start_time;
    uint64_t slot_duration;
    float    offset_estimate;
    float    float_slot_duration;
    float    inverse_of_num_slots;
} phyRadioTdma_t;

struct phyRadioPacket {
    cBuffer_t *pkt_buffer;
    uint8_t    type; // Phy packet type, phyRadioPktType_t
    uint8_t    addr; // Destination when sending, Sender when receiving
    uint8_t    slot; // Destination slot when sending, What slot it was received on when receiving
};

struct phyRadioSyncState {
    // TODO, pehaps inform about the slot scheme.
    uint32_t       tx_slot_number;  // The current phy radio tx slot number
    phyRadioMode_t mode;            // The current phy radio mode
    uint8_t        central_address; // The address to the central device
};

/**
 * The main data type for this module
 */
typedef struct {
    // phyRadio internal states
    phyRadioInterface_t *interface;
    phyRadioSyncState_t  sync_state;

    // Internal messages and buffers
    uint8_t              my_address;
    uint8_t              sync_message_array[PHY_RADIO_SYNC_MSG_SIZE + C_BUFFER_ARRAY_OVERHEAD + HAL_RADIO_PACKET_OVERHEAD];
    cBuffer_t            sync_message_buf;
    phyRadioPacket_t     sync_packet;
    uint8_t              rx_byte_array[PHY_RADIO_RX_BUFFER_SIZE];
    cBuffer_t            rx_buffer;

    // HalRadio
    halRadio_t          hal_radio_inst;
    halRadioInterface_t hal_interface;

    // Timer management
    repeating_timer_t timer;
    bool              timer_active;
    volatile bool     timer_interrupt;

    // Phy TDMA Scheduler
    phyRadioTdma_t     tdma_scheduler;
} phyRadio_t;

/**
 * Initialize the phy radio module
 * Input: phyRadio instance
 * Input: Upstream interface
 * Returns: phyRadioErr_t
 */
int32_t phyRadioInit(phyRadio_t *inst, phyRadioInterface_t *interface, uint8_t address);

/**
 * Process the phy radio
 * Input: phyRadio instance
 * Returns: phyRadioErr_t
 */
int32_t phyRadioProcess(phyRadio_t *inst);

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
 * Send a message on the next avialable slot
 * Input: phyRadio instance
 * Input: phyRadio packet
 * Returns: phyRadioErr_t
 */
int32_t phyRadioSendOnSlot(phyRadio_t *inst, phyRadioPacket_t* packet);

#endif /* PHY_RADIO_H */