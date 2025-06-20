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
#include "phy_radio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "math.h"

#ifndef PHY_RADIO_TIMER_PWM_SLICE
#define PHY_RADIO_TIMER_PWM_SLICE 0
#endif /* PHY_RADIO_TIMER_PWM_SLICE */

#ifndef PHY_RADIO_PREP_TIMER_PWM_SLICE
#define PHY_RADIO_PREP_TIMER_PWM_SLICE 1
#endif /* PHY_RADIO_PREP_TIMER_PWM_SLICE */


struct phyRadioTimerInternal {
    bool       repeating_timer_active;
    uint16_t   ticks;
    float      clk_div;
    float      clk_converter;
    float      inv_clk_converter;
    alarm_id_t task_alarm_id;    // Timer alarm used for time dependent tasks
    alarm_id_t sync_alarm_id;    // Timer alarm used for synchronization with Central

    bool       prepare_timer_active; // Timer alarm used to start the guard period
    bool       prep_single_fire; // Timer alarm used to start the guard period
    uint16_t   ticks_prep;
    phyRadioTimer_t *inst;
};

static phyRadioTimerInternal_t timer_data;

static int64_t task_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)user_data;
    inst->_private->task_alarm_id = 0;

    if (inst->task_cb != NULL) {
        inst->task_cb(inst);
    } else {
        panic("Task timer callback is NULL\n");
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

static int64_t sync_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  (phyRadioTimer_t*)user_data;
    inst->_private->sync_alarm_id = 0;

    if (inst->sync_cb != NULL) {
        inst->sync_cb(inst);
    } else {
        panic("Sync timer callback is NULL\n");
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

static void repeating_timer_callback(void) {
    // Get the current phy radio instance
    phyRadioTimer_t *inst =  timer_data.inst;

    uint32_t irq_status = pwm_get_irq0_status_mask();

    if (irq_status & (1 << PHY_RADIO_PREP_TIMER_PWM_SLICE)) {
        pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);
        if (timer_data.prep_single_fire) {
            pwm_set_irq_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
            pwm_set_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
            timer_data.prepare_timer_active = false;
        }

        if (inst->prepare_cb != NULL) {
            inst->prepare_cb(inst);
        } else {
            panic("Prepare timer callback is NULL\n");
        }
    }

    if (irq_status & (1 << PHY_RADIO_TIMER_PWM_SLICE)) {
        pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
        if (inst->repeating_cb != NULL) {
            inst->repeating_cb(inst);
        } else {
            panic("Repeating timer callback is NULL\n");
        }
    }
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
    timer_data.inst    = inst;

    // Compute the closest clockdiv to the expected slot duration with some margin
    float sys_hz = (float)clock_get_hz(clk_sys);
    timer_data.clk_div = ceilf(((PHY_RADIO_SLOT_TIME_US + PHY_RADIO_GUARD_TIME_US)/1000000.0f)/((65535.0f/sys_hz)));

    // Compute a factor that can be used to convert from us to tics
    timer_data.clk_converter = 0.000001f/(1.0f/(sys_hz/timer_data.clk_div));
    timer_data.inv_clk_converter = 1.0/timer_data.clk_converter;

    // Configure PWM slice for a period of period_us
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, timer_data.clk_div);  // 1 tick = 1 us
    pwm_init(PHY_RADIO_TIMER_PWM_SLICE, &config, false);

    // Configure PWM slice for prepare timer with same settins as periodic timer
    pwm_config_set_clkdiv(&config, timer_data.clk_div);  // 1 tick = 1 us
    pwm_init(PHY_RADIO_PREP_TIMER_PWM_SLICE, &config, false);

    uint32_t ticks = (uint32_t)PHY_RADIO_SLOT_TIME_US * timer_data.clk_converter;
    pwm_config_set_wrap(&config, ticks);
    pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), repeating_timer_callback);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
    irq_set_priority(PWM_DEFAULT_IRQ_NUM(), 0);

    pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);

    timer_data.task_alarm_id          = 0;    // Timer alarm used for time dependent tasks
    timer_data.sync_alarm_id          = 0;    // Timer alarm used for synchronization with Central
    timer_data.repeating_timer_active = false;
    timer_data.prepare_timer_active   = false;
    timer_data.prep_single_fire       = false;

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
    // Cancel active timer
    phyRadioTimerStopRepeatingTimer(inst);

    // Cancel any active alarm
    phyRadioTimerCancelPrepareTimer(inst);

    if (inst->_private->sync_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->sync_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->sync_alarm_id = 0;
        inst->sync_cb                 = NULL;
    }

    if (inst->_private->task_alarm_id != 0) {
        if (!cancel_alarm(inst->_private->task_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
        inst->_private->task_alarm_id = 0;
        inst->task_cb                 = NULL;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartSyncTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (inst->_private->sync_alarm_id == 0) {
        inst->sync_cb = cb; // This must be set before adding the alarm as a timout might occure during the add
        if ((inst->_private->sync_alarm_id = add_alarm_in_us(time_us, sync_timer_alarm_callback, inst, true)) < 0) {
            inst->_private->sync_alarm_id = 0;
            inst->sync_cb = NULL;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
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
    inst->sync_cb                 = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartTaskTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (inst->_private->task_alarm_id == 0) {
        inst->task_cb = cb; // This must be set before adding the alarm as a timout might occure during the add
        if ((inst->_private->task_alarm_id = add_alarm_in_us(time_us, task_timer_alarm_callback, inst, true)) < 0) {
            inst->_private->task_alarm_id = 0;
            inst->task_cb = NULL;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
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
    inst->task_cb                 = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartSinglePrepareTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t time_us) {
    if (timer_data.prepare_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    inst->prepare_cb            = cb;
    uint32_t ticks              = (uint32_t)(time_us * timer_data.clk_converter);
    timer_data.ticks_prep       = ticks;
    timer_data.prep_single_fire = true;

    // Start PWM counter
    pwm_set_wrap(PHY_RADIO_PREP_TIMER_PWM_SLICE, ticks);
    pwm_set_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE, 0);

    // Enable IRQ on wrap
    pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, true);
    pwm_set_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, true);

    // Save state
    timer_data.prepare_timer_active = true;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelPrepareTimer(phyRadioTimer_t *inst) {
    inst->prepare_cb = NULL;

    if (timer_data.prepare_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
        pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);
        timer_data.prepare_timer_active = false;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioPrepareTimerGetTimeToNext(phyRadioTimer_t *inst, uint32_t *period_us) {
    if (!inst->_private->prepare_timer_active) {
        *period_us = 0;
        return PHY_RADIO_TIMER_NOT_ACTIVE_ERROR;
    }

    uint16_t curr_count = pwm_get_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE);

    *period_us = (uint32_t)(timer_data.ticks - curr_count) * timer_data.inv_clk_converter;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioRepeatingTimerGetTimeToNext(phyRadioTimer_t *inst, uint32_t *period_us) {
    if (!inst->_private->repeating_timer_active) {
        *period_us = 0;
        return PHY_RADIO_TIMER_NOT_ACTIVE_ERROR;
    }

    uint16_t curr_count = pwm_get_counter(PHY_RADIO_TIMER_PWM_SLICE);

    *period_us = (uint32_t)(timer_data.ticks - curr_count) * timer_data.inv_clk_converter;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartRepeatingTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t cb, uint32_t period_us) {
    if (timer_data.repeating_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    inst->repeating_cb = cb;
    uint32_t ticks     = (uint32_t)period_us * timer_data.clk_converter;
    timer_data.ticks   = ticks;

    // Start PWM counter
    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);
    pwm_set_counter(PHY_RADIO_TIMER_PWM_SLICE, 0);

    // Enable IRQ on wrap
    pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, true);
    pwm_set_enabled(PHY_RADIO_TIMER_PWM_SLICE, true);

    // Save state
    timer_data.repeating_timer_active = true;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartCombinedTimer(phyRadioTimer_t *inst, phyRadioTimerCb_t rep_cb, phyRadioTimerCb_t prep_cb, uint32_t period_us, uint32_t guard_period) {
    if (timer_data.repeating_timer_active || timer_data.prepare_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    inst->repeating_cb = rep_cb;
    uint32_t ticks     = (uint32_t)period_us * timer_data.clk_converter;
    timer_data.ticks   = ticks;

    // Start PWM counter
    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);
    pwm_set_counter(PHY_RADIO_TIMER_PWM_SLICE, 0);

    // Enable IRQ on wrap
    pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, true);

    inst->prepare_cb      = prep_cb;
    uint32_t prep_ticks   = (uint32_t)(guard_period * timer_data.clk_converter);
    timer_data.ticks_prep = prep_ticks;

    // Set the same wrap to the prep timer
    pwm_set_wrap(PHY_RADIO_PREP_TIMER_PWM_SLICE, ticks);
    // Make sure that this timer is N ticks before the main timer, cusing it to trigger slightly before
    pwm_set_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE, prep_ticks);

    // Enable IRQ on wrap
    pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, true);


    // Start both PWM counter
    // Disable interrupts and save current state
    uint32_t irq_status = save_and_disable_interrupts();
    uint32_t mask = (1u << PHY_RADIO_TIMER_PWM_SLICE) | (1u << PHY_RADIO_PREP_TIMER_PWM_SLICE);
    pwm_set_mask_enabled(mask);
    restore_interrupts(irq_status);

    // Save state
    timer_data.prep_single_fire       = false;
    timer_data.prepare_timer_active   = true;
    timer_data.repeating_timer_active = true;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStopCombinedTimer(phyRadioTimer_t *inst) {
    if (timer_data.repeating_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
        timer_data.repeating_timer_active = false;
    }

    inst->repeating_cb = NULL;


    if (timer_data.prepare_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_PREP_TIMER_PWM_SLICE, false);
        pwm_clear_irq(PHY_RADIO_PREP_TIMER_PWM_SLICE);
        timer_data.prepare_timer_active = false;
    }

    inst->prepare_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerUpdateCombinedTimer(phyRadioTimer_t *inst, float new_period_us) {
    if (!timer_data.repeating_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    uint32_t ticks = (uint32_t)(new_period_us * timer_data.clk_converter);
    timer_data.ticks = ticks;

    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);
    pwm_set_wrap(PHY_RADIO_PREP_TIMER_PWM_SLICE, ticks);

    // This is a critical section
    uint32_t irq_status       = save_and_disable_interrupts();
    uint16_t rep_timer_count  = pwm_get_counter(PHY_RADIO_TIMER_PWM_SLICE);
    uint16_t prep_timer_count = pwm_get_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE);
    restore_interrupts(irq_status);

    int32_t diff = 0;
    if (rep_timer_count > prep_timer_count) {
        // This occurs when the prep has wrapped but the prep has not wrapped
        diff = ticks - rep_timer_count;
        diff += prep_timer_count;
    } else if (rep_timer_count < prep_timer_count) {
        // Both timers have wraped for this session
        diff = prep_timer_count - rep_timer_count;
    } else {
        // This would be fatal, very bad there is no distance between them any more!
        phyRadioTimerStopCombinedTimer(inst);
        return PHY_RADIO_TIMER_ERROR;
    }

    // Calculate if there has been any drift between the timers
    diff = diff - timer_data.ticks_prep;

    if (diff != 0) {
        // This is a critical section
        irq_status = save_and_disable_interrupts();
        // Add or subtrackt a little bit from the prep timer
        prep_timer_count = pwm_get_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE);
        prep_timer_count -= diff;
        pwm_set_counter(PHY_RADIO_PREP_TIMER_PWM_SLICE, prep_timer_count);
        restore_interrupts(irq_status);
    } else {
        // It is perfectly in line, great!
        // Do nothing
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStopRepeatingTimer(phyRadioTimer_t *inst) {
    if (timer_data.repeating_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        timer_data.repeating_timer_active = false;
    }

    inst->repeating_cb = NULL;
    return PHY_RADIO_TIMER_SUCCESS;
}

/**
 * Update the PWM period on the fly
 */
int32_t phyRadioTimerUpdateRepeatingTimer(phyRadioTimer_t *inst, float new_period_us) {
    if (!timer_data.repeating_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    uint32_t ticks = (uint32_t)(new_period_us * timer_data.clk_converter);
    timer_data.ticks = ticks;

    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);
    return PHY_RADIO_TIMER_SUCCESS;
}
