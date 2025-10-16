/**
 * @file:       phy_radio_timer.h
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Header file for Phy radio firmware timer wrapper
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

#ifndef PHY_RADIO_TIMER_H
#define PHY_RADIO_TIMER_H

#include "pico/stdlib.h"

typedef enum {
    PHY_RADIO_TIMER_ACTIVE            = 1,
    PHY_RADIO_TIMER_SUCCESS           = 0,
    PHY_RADIO_TIMER_NULL_ERROR        = -21001,
    PHY_RADIO_TIMER_GEN_ERROR         = -21002,
    PHY_RADIO_TIMER_HAL_ERROR         = -21003,
    PHY_RADIO_TIMER_DOUBLE_INIT_ERROR = -21004,
    PHY_RADIO_TIMER_ACTIVE_ERROR      = -21005,
    PHY_RADIO_TIMER_NOT_ACTIVE_ERROR  = -21006,
} phyRadioTimerErr_t;

typedef struct phyRadioTimer phyRadioTimer_t;
typedef struct phyRadioTimerConfig phyRadioTimerConfig_t;
typedef struct phyRadioTaskTimer phyRadioTaskTimer_t;

typedef struct phyRadioTimerInternal phyRadioTimerInternal_t;

typedef int32_t (*phyRadioFrameTimerCb_t)(phyRadioTimer_t *inst);
typedef int32_t (*phyRadioTickTimerCb_t)(phyRadioTimer_t *inst, uint16_t index);
typedef int32_t (*phyRadioTaskTimerCb_t)(phyRadioTaskTimer_t *inst);

struct phyRadioTimer {
    bool                     initialized;
    phyRadioTimerInternal_t *_private;

    phyRadioFrameTimerCb_t      frame_cb;
    phyRadioTickTimerCb_t      tick_cb;

    // Tick sequence configurations
    uint16_t               tick_index;
    phyRadioTimerConfig_t *timer_config;
};

struct phyRadioTimerConfig {
    uint16_t *tick_sequence; // Array of tick intervals
    uint16_t  num_ticks;     // Number of elements in the array
};

struct phyRadioTaskTimer {
    bool                  initialized;
    alarm_id_t            task_alarm_id;    // Timer alarm used for time dependent tasks
    phyRadioTaskTimerCb_t task_cb;
};

/**
 * Init the phy radio timer
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerInit(phyRadioTimer_t *inst, uint32_t max_frame_len_us, uint32_t max_tick_len_us);

/**
 * Set a tick timer sequence configuration
 * Input: phyRadioTimer instance
 * Input: configuration, NOTE this parameter must persist through the lifetime of the timer instance
 *        the data stored in the configuration will be updated. Only call this function once for a given config.
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerSetConfig(phyRadioTimer_t *inst, phyRadioTimerConfig_t *config);

/**
 * De-Init the phy radio timer
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerDeInit(phyRadioTimer_t *inst);

/**
 * Cancel all active phy radio timers
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerCancelAll(phyRadioTimer_t *inst);

/**
 * Stop an ongoing tick timer without complete disable
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStopTickTimer(phyRadioTimer_t *inst);

/**
 * Start both a frame timer and a tick timer that triggers at a regular interval after the frame timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on each timer tick
 * Input: Callback to be called on each new frame
 * Input: Frame timer period
 * Input: Tick timer period
 * Input: Frame timer offset (starts frame timer this many us into the period)
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartCombinedTimer(phyRadioTimer_t *inst, phyRadioTickTimerCb_t tick_cb, phyRadioFrameTimerCb_t frame_cb, uint32_t period_us, uint32_t tick_period_us, uint32_t frame_offset_us);

/**
 * Update the period synchronously on both frame timers keeping the tick timer
 * Input: phyRadioTimer instance
 * Input: New Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerUpdateCombinedTimer(phyRadioTimer_t *inst, float new_period_us);

/**
 * Stop both frame timer and tick timer
 * Input: phyRadioTimer instance
 * Input: New Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStopCombinedTimer(phyRadioTimer_t *inst);

/**
 * Convert tick timer ticks to us
 * Input: phyRadioTimer instance
 * Input: Number of ticks
 * Returns: phyRadioTimerErr_t or ticks
 */
int32_t phyRadioTickTimerTickToUs(phyRadioTimer_t *inst, uint16_t ticks);

/**
 * Check if the frame timer is active
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t or ticks
 */
int32_t phyRadioTimerIsFrameTimerActive(phyRadioTimer_t *inst);

/**
 * Init a task timer instance
 * Input: phyRadioTaskTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTaskTimerInit(phyRadioTaskTimer_t *inst);

/**
 * Start a task timer
 * Input: phyRadioTaskTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartTaskTimer(phyRadioTaskTimer_t *inst, phyRadioTaskTimerCb_t cb, uint32_t time_us);

/**
 * Cancel an ongoing task timer
 * Input: phyRadioTaskTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerCancelTaskTimer(phyRadioTaskTimer_t *inst);

#endif /* PHY_RADIO_TIMER_H */