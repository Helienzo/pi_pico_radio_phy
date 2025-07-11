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
    PHY_RADIO_TIMER_SUCCESS           = 0,
    PHY_RADIO_TIMER_NULL_ERROR        = -21001,
    PHY_RADIO_TIMER_GEN_ERROR         = -21002,
    PHY_RADIO_TIMER_HAL_ERROR         = -21003,
    PHY_RADIO_TIMER_DOUBLE_INIT_ERROR = -21004,
    PHY_RADIO_TIMER_ACTIVE_ERROR      = -21005,
    PHY_RADIO_TIMER_NOT_ACTIVE_ERROR  = -21006,
} phyRadioTimerErr_t;

typedef struct phyRadioTimer phyRadioTimer_t;
typedef struct phyRadioTimerInternal phyRadioTimerInternal_t;

typedef int32_t (*phyRadioTimerCb_t)(phyRadioTimer_t *inst);

struct phyRadioTimer {
    bool                     initialized;
    phyRadioTimerInternal_t *_private;

    phyRadioTimerCb_t prepare_cb;
    phyRadioTimerCb_t sync_cb;
    phyRadioTimerCb_t task_cb;
    phyRadioTimerCb_t repeating_cb;
};

/**
 * Init the phy radio timer
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerInit(phyRadioTimer_t *inst);

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
 * Start a sync timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartSyncTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us);

/**
 * Cancel an ongoing sync timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerCancelSyncTimer(phyRadioTimer_t *inst);

/**
 * Start a task timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartTaskTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us);

/**
 * Cancel an ongoing task timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerCancelTaskTimer(phyRadioTimer_t *inst);

/**
 * Start a prepare timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartSinglePrepareTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us);

/**
 * Cancel an ongoing prepare timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerCancelPrepareTimer(phyRadioTimer_t *inst);

/**
 * Start a repeating timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on each timeout
 * Input: Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartRepeatingTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t period_us);

/**
 * Stop an ongoing repeating timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on timeout
 * Input: Timeout time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStopRepeatingTimer(phyRadioTimer_t *inst);

/**
 * Update the period on the repeating timer
 * Input: phyRadioTimer instance
 * Input: Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerUpdateRepeatingTimer(phyRadioTimer_t *inst, float new_period_us);

/**
 * Get time until next repeating timer
 * Input: phyRadioTimer instance
 * Input: Time until next fire
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioRepeatingTimerGetTimeToNext(phyRadioTimer_t *inst, uint32_t *period_us);

/**
 * Get time until next prepare timer
 * Input: phyRadioTimer instance
 * Input: Time until next fire
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioPrepareTimerGetTimeToNext(phyRadioTimer_t *inst, uint32_t *period_us);

/**
 * Start both a repeating timer and a prepare timer that lauches guard period before repeating timer
 * Input: phyRadioTimer instance
 * Input: Callback to be called on repeating timer
 * Input: Callback to be called on prep timer
 * Input: Shared timer period
 * Input: Guard period between timers
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStartCombinedTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t rep_cb, phyRadioTimerCb_t prep_cb, uint32_t period_us, uint32_t guard_period);

/**
 * Update the period synchronously on both repeating timers keeping the guard period
 * Input: phyRadioTimer instance
 * Input: New Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerUpdateCombinedTimer(phyRadioTimer_t *inst, float new_period_us);

/**
 * Stop both repeating timers
 * Input: phyRadioTimer instance
 * Input: New Period time in micro seconds
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerStopCombinedTimer(phyRadioTimer_t *inst);
#endif /* PHY_RADIO_TIMER_H */