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
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "math.h"

#ifndef PHY_RADIO_TIMER_PWM_SLICE
#define PHY_RADIO_TIMER_PWM_SLICE 0
#endif /* PHY_RADIO_TIMER_PWM_SLICE */

#ifndef PHY_RADIO_TICK_TIMER_PWM_SLICE
#define PHY_RADIO_TICK_TIMER_PWM_SLICE 1
#endif /* PHY_RADIO_TICK_TIMER_PWM_SLICE */


struct phyRadioTimerInternal {
    bool       frame_timer_active;
    uint16_t   ticks;
    float      clk_div;
    float      clk_converter;
    float      inv_clk_converter;

    float      tick_clk_div;
    float      tick_clk_converter;
    float      tick_inv_clk_converter;

    bool       tick_timer_active; // Timer alarm used to start the guard period
    uint16_t   ticks_tick_timer;
    phyRadioTimer_t *inst;
};

static phyRadioTimerInternal_t timer_data;

static int64_t task_timer_alarm_callback(alarm_id_t id, void *user_data) {
    // Get the current phy radio instance
    phyRadioTaskTimer_t *inst =  (phyRadioTaskTimer_t*)user_data;
    inst->task_alarm_id = 0;

    if (inst->task_cb != NULL) {
        inst->task_cb(inst);
    } else {
        panic("Task timer callback is NULL\n");
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

static void frame_timer_callback(void) {
    // Get the current phy radio instance
    phyRadioTimer_t       *inst =  timer_data.inst;
    phyRadioTimerConfig_t *timer_config = inst->timer_config;

    uint32_t irq_status = pwm_get_irq0_status_mask();

    if (irq_status & (1 << PHY_RADIO_TICK_TIMER_PWM_SLICE)) {
        pwm_clear_irq(PHY_RADIO_TICK_TIMER_PWM_SLICE);

        // Update wrap number for tick timer
        if (inst->tick_index < timer_config->num_ticks) {
            pwm_set_wrap(PHY_RADIO_TICK_TIMER_PWM_SLICE, timer_config->tick_sequence[inst->tick_index]);
            inst->tick_index++;
        } else {
            // Increment the index to keep track of number of interrupts
            inst->tick_index++;
            // Disable the PWM timer, sequence of interrupts are done
            pwm_set_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, false);
        }

        if (inst->tick_cb != NULL) {
            inst->tick_cb(inst, inst->tick_index - 3);
        } else {
            panic("Tick timer callback is NULL\n");
        }
    }

    if (irq_status & (1 << PHY_RADIO_TIMER_PWM_SLICE)) {
        pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);

        // Start the tick timer
        pwm_set_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, false);
        // Set the first wrap value
        pwm_set_wrap(PHY_RADIO_TICK_TIMER_PWM_SLICE, timer_config->tick_sequence[0]);
        pwm_set_counter(PHY_RADIO_TICK_TIMER_PWM_SLICE, 0);
        // Start the timer
        pwm_set_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, true);
        // Pre-Load the next wrap value that will be latched on the next wrap
        pwm_set_wrap(PHY_RADIO_TICK_TIMER_PWM_SLICE, timer_config->tick_sequence[1]);
        // Set the index of the next value to be loaded
        inst->tick_index = 2;

        if (inst->frame_cb != NULL) {
            inst->frame_cb(inst);
        } else {
            panic("Frame timer callback is NULL\n");
        }
    }
}

/**
 * To trigger the PWM interrupt sequence using IRQ, and PWM double buffered TOP value:
 * 1. Manually write the wrap value TOP0 to the PWM
 * 2. Start the PWM
 * 3. Manually Buffer the wrap value TOP1 to the PWM
 * 4. Set the index to point to wrap value TOP2
 * 
 * [0-TOP0] Now on the first wrap the buffered value TOP1 is written to the PWM, Write the TOP2 value to PWM buffer, increment index
 * [0-TOP1] The second wrap triggers after TOP1, TOP2 is written to the PWM, Write the TOP3 value to PWM buffer, increment index
 * [0-TOP2] The third wrap triggers after TOP2, TOP3 is written to the PWM, Write the TOP4 value to PWM buffer, increment index
 * ..
 * ..
 */

int32_t phyRadioTaskTimerInit(phyRadioTaskTimer_t *inst) {
    if (inst == NULL) {
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    if (inst->initialized) {
        return PHY_RADIO_TIMER_DOUBLE_INIT_ERROR;
    }

    inst->initialized   = true;
    inst->task_cb       = NULL;
    inst->task_alarm_id = 0;

    return PHY_RADIO_TIMER_SUCCESS;
}
/**
 * Init the phy radio timer
 * Input: phyRadioTimer instance
 * Returns: phyRadioTimerErr_t
 */
int32_t phyRadioTimerInit(phyRadioTimer_t *inst, uint32_t max_frame_len_us, uint32_t max_tick_len_us) {
    if (inst == NULL) {
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    if (inst->initialized) {
        return PHY_RADIO_TIMER_DOUBLE_INIT_ERROR;
    }

    inst->initialized  = true;
    inst->tick_cb      = NULL;
    inst->frame_cb     = NULL;
    inst->timer_config = NULL;
    inst->_private     = &timer_data;
    timer_data.inst    = inst;

    // Compute the closest clockdiv to the expected maximum frame duration
    float sys_hz = (float)clock_get_hz(clk_sys);
    timer_data.clk_div = ceilf(((max_frame_len_us)/1000000.0f)/((65535.0f/sys_hz)));

    // Compute a factor that can be used to convert from us to tics
    timer_data.clk_converter     = 0.000001f/(1.0f/(sys_hz/timer_data.clk_div));
    timer_data.inv_clk_converter = 1.0/timer_data.clk_converter;

    // Configure PWM slice for a period of period_us
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, timer_data.clk_div);  // 1 tick = 1 us
    pwm_init(PHY_RADIO_TIMER_PWM_SLICE, &config, false);

    // Set a defualt value for number of ticks
    uint32_t ticks = (uint32_t)(max_frame_len_us * timer_data.clk_converter);
    pwm_config_set_wrap(&config, ticks);

    // Clear the IRQ for the frame timer
    pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);

    // Configure PWM slice for tick timer
    config = pwm_get_default_config();
    // Compute the closest clockdiv to the expected maximum tick duration
    timer_data.tick_clk_div = ceilf(((max_tick_len_us)/1000000.0f)/((65535.0f/sys_hz)));

    // Compute a factor that can be used to convert from us to tics
    timer_data.tick_clk_converter     = 0.000001f/(1.0f/(sys_hz/timer_data.tick_clk_div));
    timer_data.tick_inv_clk_converter = 1.0/timer_data.tick_clk_converter;

    // Configure PWM slice for the tick timer
    pwm_config_set_clkdiv(&config, timer_data.tick_clk_div);  // 1 tick = 1 us
    pwm_init(PHY_RADIO_TICK_TIMER_PWM_SLICE, &config, false);

    // Set a defualt value for number of ticks
    ticks = (uint32_t)(max_tick_len_us * timer_data.tick_clk_converter);
    pwm_config_set_wrap(&config, ticks);

    // Clear the IRQ for the tick timer
    pwm_clear_irq(PHY_RADIO_TICK_TIMER_PWM_SLICE);

    // Configure the IRQ
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), frame_timer_callback);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
    irq_set_priority(PWM_DEFAULT_IRQ_NUM(), 0);

    timer_data.frame_timer_active = false;
    timer_data.tick_timer_active  = false;

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
    return phyRadioTimerStopCombinedTimer(inst);
}

int32_t phyRadioTimerStartTaskTimer(phyRadioTaskTimer_t *inst, phyRadioTaskTimerCb_t cb, uint32_t time_us) {
    if (inst->task_alarm_id == 0) {
        inst->task_cb = cb; // This must be set before adding the alarm as a timout might occure during the add
        if ((inst->task_alarm_id = add_alarm_in_us(time_us, task_timer_alarm_callback, inst, true)) < 0) {
            inst->task_alarm_id = 0;
            inst->task_cb       = NULL;
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    } else {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerCancelTaskTimer(phyRadioTaskTimer_t *inst) {
    if (inst->task_alarm_id != 0) {
        if (!cancel_alarm(inst->task_alarm_id)) {
            return PHY_RADIO_TIMER_HAL_ERROR;
        }
    }

    inst->task_alarm_id = 0;
    inst->task_cb       = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStopTickTimer(phyRadioTimer_t *inst) {
    pwm_set_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, false);
    pwm_set_counter(PHY_RADIO_TICK_TIMER_PWM_SLICE, 0);

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerSetConfig(phyRadioTimer_t *inst, phyRadioTimerConfig_t *config) {
    if (inst == NULL || config == NULL || config->tick_sequence == NULL) {
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    // Convert the tick sequence from us to timer ticks
    for (uint32_t i = 0; i < config->num_ticks; i++) {
        config->tick_sequence[i] = config->tick_sequence[i] * timer_data.tick_clk_converter;
    }

    // Store the config
    inst->timer_config = config;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStartCombinedTimer(phyRadioTimer_t *inst, phyRadioTickTimerCb_t tick_cb, phyRadioFrameTimerCb_t frame_cb, uint32_t period_us, uint32_t tick_period_us, uint32_t frame_offset_us) {
    if (timer_data.frame_timer_active || timer_data.tick_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    if (inst->timer_config == NULL) {
        // It is not possible to start the timer without a valid configuration
        return PHY_RADIO_TIMER_NULL_ERROR;
    }

    inst->frame_cb     = frame_cb;
    uint32_t ticks     = (uint32_t)period_us * timer_data.clk_converter;
    timer_data.ticks   = ticks;

    // Start PWM counter with offset
    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);
    uint32_t frame_offset_ticks = (uint32_t)(frame_offset_us * timer_data.clk_converter);
    pwm_set_counter(PHY_RADIO_TIMER_PWM_SLICE, frame_offset_ticks);

    // Enable IRQ on wrap
    pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, true);

    inst->tick_cb               = tick_cb;
    uint32_t tick_ticks         = (uint32_t)(tick_period_us * timer_data.tick_clk_converter);
    timer_data.ticks_tick_timer = tick_ticks;

    // Configure the tick timer but don't start it yet
    pwm_set_wrap(PHY_RADIO_TICK_TIMER_PWM_SLICE, tick_ticks);
    pwm_set_counter(PHY_RADIO_TICK_TIMER_PWM_SLICE, 0);

    // Enable IRQ on wrap but don't start the timer
    pwm_clear_irq(PHY_RADIO_TICK_TIMER_PWM_SLICE);
    pwm_set_irq_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, true);

    // Start only the frame timer - tick timer will be started by frame timer callback
    pwm_set_enabled(PHY_RADIO_TIMER_PWM_SLICE, true);

    timer_data.tick_timer_active      = true;
    timer_data.frame_timer_active     = true;

    gpio_put(HAL_RADIO_PIN_TX_RX, 0);

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerStopCombinedTimer(phyRadioTimer_t *inst) {
    if (timer_data.frame_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_TIMER_PWM_SLICE, false);
        pwm_clear_irq(PHY_RADIO_TIMER_PWM_SLICE);
        timer_data.frame_timer_active = false;
    }

    inst->frame_cb = NULL;


    if (timer_data.tick_timer_active) {
        pwm_set_irq_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, false);
        pwm_set_enabled(PHY_RADIO_TICK_TIMER_PWM_SLICE, false);
        pwm_clear_irq(PHY_RADIO_TICK_TIMER_PWM_SLICE);
        timer_data.tick_timer_active = false;
    }

    inst->tick_cb = NULL;

    return PHY_RADIO_TIMER_SUCCESS;
}

int32_t phyRadioTimerUpdateCombinedTimer(phyRadioTimer_t *inst, float new_period_us) {
    if (!timer_data.frame_timer_active) {
        return PHY_RADIO_TIMER_ACTIVE_ERROR;
    }

    uint32_t ticks = (uint32_t)(new_period_us * timer_data.clk_converter);
    timer_data.ticks = ticks;

    // Only update the frame timer period - tick timer period stays the same
    pwm_set_wrap(PHY_RADIO_TIMER_PWM_SLICE, ticks);

    return PHY_RADIO_TIMER_SUCCESS;
}