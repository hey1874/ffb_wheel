# CMakeLists.txt - FFB wheel firmware for RP2040
#
# Prerequisites (one-time):
#   brew install arm-none-eabi-gcc cmake ninja
# Build:
#   cmake -B build -G Ninja
#   cmake --build build
# Flash: copy build/ffb_wheel.uf2 to the Pico (BOOTSEL mode).

cmake_minimum_required(VERSION 3.13)

# ---- Pico SDK (fetch from git, tag sdk-2.0.0) ----
set(PICO_SDK_FETCH_FROM_GIT ON)
set(PICO_SDK_FETCH_FROM_GIT_TAG 2.1.1)
include(pico_sdk_import.cmake)

project(ffb_wheel C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---- Firmware ----
add_executable(ffb_wheel
    src/main.c
    src/ffb.c
    src/usb_descriptors.c
    src/mcp2515.c      # MCP2515 SPI-CAN driver
    src/odrive_can.c   # ODrive CAN protocol (torque + encoder polling)
    src/pedals.c       # Analog pedals (throttle/brake/clutch) via on-chip ADC
    src/buttons.c      # Momentary buttons via GPIO (debounced, up to 32)
)

target_include_directories(ffb_wheel PRIVATE src)

# ---- Build-time tuning knobs ----
# Forward -D<NAME>=<val> from the cmake command line to the compiler. Without
# this, `cmake -DMAX_NM=3.0` only sets an unused CMake cache variable and the C
# #ifndef default silently wins. (BUTTON_GPIOS is a brace list — cmake mangles
# the commas — so set that one in buttons.h, not via -D.)
foreach(_opt
    MAX_NM WHEEL_MAX_TURNS ENCODER_SIGN TORQUE_SIGN WHEEL_CENTER_AT_BOOT
    ODRIVE_NODE_ID MCP_BAUD MCP_OSC_HZ MCP_SPI_HZ
    MCP_SCK_PIN MCP_MOSI_PIN MCP_MISO_PIN MCP_CS_PIN
    PEDAL_COUNT PEDAL_ADC_BASE_GPIO PEDAL_INVERT_MASK
    BUTTON_CHIPS BUTTON_SDA_PIN BUTTON_SCL_PIN BUTTON_I2C_HZ
    BUTTON_ADDR0 BUTTON_ADDR1 BUTTON_DEBOUNCE)
  if(DEFINED ${_opt})
    target_compile_definitions(ffb_wheel PRIVATE ${_opt}=${${_opt}})
  endif()
endforeach()

# tinyusb_device: TinyUSB device stack (uses our src/tusb_config.h).
# tinyusb_board:   board_init/led/serial glue for RP2040.
# hardware_spi:    Pico-SDK SPI driver (used by mcp2515.c).
# hardware_gpio:   Pico-SDK GPIO driver (CS pin, used by mcp2515.c).
target_link_libraries(ffb_wheel
    pico_stdlib
    pico_multicore    # core1 runs the CAN link (main.c core1_entry)
    tinyusb_device
    tinyusb_board
    hardware_timer
    hardware_spi
    hardware_gpio
    hardware_adc      # on-chip ADC for analog pedals (pedals.c)
    hardware_i2c      # MCP23017 button expander(s) (buttons.c)
)

# Emit .uf2, .hex, .disasm
pico_add_extra_outputs(ffb_wheel)

# Enable USB (Pico-SDK default is on for pico board; ensure explicit).
pico_enable_stdio_usb(ffb_wheel 0)
pico_enable_stdio_uart(ffb_wheel 0)
