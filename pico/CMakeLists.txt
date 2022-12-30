cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(OpenThermOstat VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w)
pico_sdk_init()

set(WIFI_SSID "my-wifi" CACHE STRING "Wifi network SSID")
set(WIFI_PASSWORD "my-wifi-password" CACHE STRING "Wifi network password")
set(WIFI_AUTH "CYW43_AUTH_WPA2_AES_PSK" CACHE STRING "Wifi authentication method")

add_executable(openthermostat)

pico_enable_stdio_usb(openthermostat 0)
pico_enable_stdio_uart(openthermostat 1)
pico_generate_pio_header(openthermostat ${CMAKE_CURRENT_LIST_DIR}/opentherm.pio)

target_sources(openthermostat PUBLIC main.cpp ${PICO_SDK_PATH}/lib/lwip/src/apps/mqtt/mqtt.c)

target_include_directories(openthermostat PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../include)

target_link_libraries(openthermostat PRIVATE
  pico_stdlib
  pico_multicore
  pico_platform
  hardware_adc
  hardware_pio
  hardware_flash
  hardware_exception
  pico_cyw43_arch_lwip_threadsafe_background
  pico_lwip_http)

target_compile_definitions(openthermostat PRIVATE
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  WIFI_AUTH=${WIFI_AUTH})

target_compile_options(openthermostat PRIVATE -Wall)

pico_add_extra_outputs(openthermostat)