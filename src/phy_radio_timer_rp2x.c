/**
 * @file:       phy_radio_timer_rp2x.c
 * @author:     Lucas Wennerholm <lucas.wennerholm@gmail.com>
 * @brief:      Implementation file for Phy radio firmware timer wrapper specifically for the rp2350 MCU
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

#include "phy_radio_timer.h"
#include "pico/stdlib.h"
#include "pico/time.h"

struct phyRadioTimerInternal {
    repeating_timer_t repeating_timer;
    bool              repeating_timer_active;
    alarm_id_t        task_alarm_id;    // Timer alarm used for time dependent tasks
    alarm_id_t        sync_alarm_id;    // Timer alarm used for synchronization with Central
    alarm_id_t        prepare_alarm_id; // Timer alarm used to start the guard period
};

static phyRadioTimerInternal_t timer_data;

static int64_t task_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)user_data;
    inst->_private->task_alarm_id = 0;

    inst->task_cb(inst);

    return PHY_RADIO_TIMER_SUCCESS;
}

static int64_t sync_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)user_data;
    inst->_private->sync_alarm_id = 0;

    inst->sync_cb(inst);

    return PHY_RADIO_TIMER_SUCCESS;
}

static int64_t prepare_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)user_data;
    inst->_private->prepare_alarm_id = 0;

    inst->prepare_cb(inst);

    return PHY_RADIO_TIMER_SUCCESS;
}

static bool repeating_timer_callback(__unused struct repeating_timer *t) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)t->user_data;

    inst->repeating_cb(inst);

    return true;
}

/**
 * Init the phy radio timer
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerInit(phyRadioTimer_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    if (inst->initialized) {
        return PHY_RADIO_TIMER_DOUBLE_INIT_ERROR;
    }

    inst->initialized = true;

    inst->prepare_cb   = NULL;
    inst->sync_cb      = NULL;
    inst->task_cb      = NULL;
    inst->repeating_cb = NULL;
    inst->_private     = &timer_data;

    timer_data.task_alarm_id          = 0;    // Timer alarm used for time dependent tasks
    timer_data.prepare_alarm_id       = 0; // Timer alarm used to start the guard period
    timer_data.sync_alarm_id          = 0;    // Timer alarm used for synchronization with Central
    timer_data.repeating_timer_active = false;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerDeInit(phyRadioTimer_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    inst->initialized = false;

    int32_t res = phyRadioTimerCancelAll(inst);
    if (res != PHY_RADIO_TIMER_SUCCESS) {
        return res;
    }

    inst->_private    = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelAll(phyRadioTimer_t *inst) {
    inst->sync_cb = NULL;
    inst->task_cb = NULL;
    inst->prepare_cb = NULL;
    inst->repeating_cb = NULL;

    // Cancel active timer
    if (inst->_private->repeating_timer_active) {
        if (!cancel_repeating_timer(&inst->_private->repeating_timer)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->repeating_timer_active = false;
    }

    // Cancel any active alarm
    if (inst->_private->prepare_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->prepare_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->prepare_alarm_id = 0;
    }

    if (inst->_private->sync_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->sync_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->sync_alarm_id = 0;
    }

    if (inst->_private->task_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->task_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->task_alarm_id = 0;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartSyncTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (inst->_private->sync_alarm_id == 0) {
        if ((inst->_private->sync_alarm_id = add_alarm_in_us(time_us, sync_timer_alarm_callback, inst, true)) < 0) {
            inst->_private->sync_alarm_id = 0;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->sync_cb = cb;
    } else {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelSyncTimer(phyRadioTimer_t *inst) {
    if (inst->_private->sync_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->sync_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    }

    inst->_private->sync_alarm_id = 0;
    inst->sync_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartTaskTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (inst->_private->task_alarm_id == 0) {
        if ((inst->_private->task_alarm_id = add_alarm_in_us(time_us, task_timer_alarm_callback, inst, true)) < 0) {
            inst->_private->task_alarm_id = 0;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->task_cb = cb;
    } else {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelTaskTimer(phyRadioTimer_t *inst) {
    if (inst->_private->task_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->task_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    }

    inst->_private->task_alarm_id = 0;
    inst->task_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartPrepareTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (inst->_private->prepare_alarm_id == 0) {
        if ((inst->_private->prepare_alarm_id = add_alarm_in_us(time_us, prepare_timer_alarm_callback, inst, true)) < 0) {
            inst->_private->prepare_alarm_id = 0;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->prepare_cb = cb;
    } else {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelPrepareTimer(phyRadioTimer_t *inst) {
    if (inst->_private->prepare_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->prepare_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    }

    inst->_private->prepare_alarm_id = 0;
    inst->prepare_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartRepeatingTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t period_us) {
    if (!add_repeating_timer_us(period_us, repeating_timer_callback, inst, &inst->_private->repeating_timer)) {
        return PHY_RADIO_TIMER_HAL_ERROR;
    }

    inst->repeating_cb = cb;
    inst->_private->repeating_timer_active = true;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStopRepeatingTimer(phyRadioTimer_t *inst) {
    if (inst->_private->repeating_timer_active) {
        if (!cancel_repeating_timer(&inst->_private->repeating_timer)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    }

    inst->_private->repeating_timer_active = false;
    inst->repeating_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerUpdateRepeatingTimer(phyRadioTimer_t *inst, uint32_t new_period_us) {
    return PHY_RADIO_TIMER_SUCCESS;
}