# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(radio_phy_tdd_peripheral C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
FetchContent_Declare(
  rp2x_rfm69
  GIT_REPOSITORY https://github.com/Helienzo/rp2x_rfm69.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(rp2x_rfm69)

FetchContent_Declare(
  c_buffer
  GIT_REPOSITORY https://github.com/Helienzo/c_buffer.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(c_buffer)

FetchContent_Declare(
  pi_pico_gpio_hal
  GIT_REPOSITORY https://github.com/Helienzo/pi_pico_gpio_hal.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(pi_pico_gpio_hal)

FetchContent_Declare(
  pico_bootsel_button
  GIT_REPOSITORY https://github.com/Helienzo/pico_bootsel_button.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(pico_bootsel_button)

FetchContent_Declare(
  static_queue  
  GIT_REPOSITORY https://github.com/Helienzo/static_queue.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(static_queue)

FetchContent_Declare(
  rfm69_radio_hal
  GIT_REPOSITORY https://github.com/Helienzo/rfm69_radio_hal.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(rfm69_radio_hal)

set(MODULE_PATH "../..")
add_subdirectory(${MODULE_PATH} libs/pi_pico_radio_phy)

# Add executable. Default name is the project name, version 0.1

add_executable(radio_phy_tdd_peripheral radio_phy_tdd_peripheral.c )

pico_set_program_name(radio_phy_tdd_peripheral "radio_phy_tdd_peripheral")
pico_set_program_version(radio_phy_tdd_peripheral "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(radio_phy_tdd_peripheral 1)
pico_enable_stdio_usb(radio_phy_tdd_peripheral 1)

# Add the standard library to the build
target_link_libraries(radio_phy_tdd_peripheral
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(radio_phy_tdd_peripheral PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(radio_phy_tdd_peripheral 
        hardware_pwm
        hardware_spi
        rp2x_rfm69_lib
        c_buffer
        pi_pico_gpio_hal
        pico_bootsel_button
        static_queue
        rfm69_radio_hal
        pi_pico_radio_phy
        )

pico_add_extra_outputs(radio_phy_tdd_peripheral)

