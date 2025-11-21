#ifndef PHY_RADIO_TIMER_DEFAULT_CONFIG_H
#define PHY_RADIO_TIMER_DEFAULT_CONFIG_H

#include <stdint.h>

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

#endif /* PHY_RADIO_TIMER_DEFAULT_CONFIG_H */