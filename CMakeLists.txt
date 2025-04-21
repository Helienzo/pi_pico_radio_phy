cmake_minimum_required(VERSION 3.13)

project(pi_pico_radio_phy C ASM)

add_library(pi_pico_radio_phy INTERFACE)

target_sources(pi_pico_radio_phy INTERFACE
	src/phy_radio.c
	src/phy_radio_timer_rp2x.c
)

target_include_directories(pi_pico_radio_phy INTERFACE
	src
)
