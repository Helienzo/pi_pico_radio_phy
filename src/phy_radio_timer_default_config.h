#ifndef PHY_RADIO_TIMER_DEFAULT_CONFIG_H
#define PHY_RADIO_TIMER_DEFAULT_CONFIG_H

#include <stdint.h>
#include "phy_radio_timer.h"

typedef enum {
    PHY_RADIO_TIMER_FIRST_SLOT_GUARD_START,
    PHY_RADIO_TIMER_SLOT_GUARD_START,
    PHY_RADIO_TIMER_SLOT_START,
    PHY_RADIO_TIMER_SLOT_END_GUARD,
} phyRadioFrameIndex_t;

struct phyRadioTimerConfig {
    uint16_t *tick_sequence; // Array of tick intervals
    uint16_t  num_ticks;     // Number of elements in the array
};

// Configure the frame
static uint32_t phyRadioFrameConfig(phyRadioTimerConfig_t *config, phyRadioFrameConfig_t *frame, uint32_t frame_lenght, uint16_t *ticks, uint32_t num_ticks) {
    config->num_ticks     = num_ticks;
    config->tick_sequence = ticks;

    return frame_lenght;
}

#endif /* PHY_RADIO_TIMER_DEFAULT_CONFIG_H */