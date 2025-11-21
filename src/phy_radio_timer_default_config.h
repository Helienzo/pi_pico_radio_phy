#ifndef PHY_RADIO_TIMER_DEFAULT_CONFIG_H
#define PHY_RADIO_TIMER_DEFAULT_CONFIG_H

#include <stdint.h>

struct phyRadioTimerConfig {
    uint16_t *tick_sequence; // Array of tick intervals
    uint16_t  num_ticks;     // Number of elements in the array
};

#endif /* PHY_RADIO_TIMER_DEFAULT_CONFIG_H */