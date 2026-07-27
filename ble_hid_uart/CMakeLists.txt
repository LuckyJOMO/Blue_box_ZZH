# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/rcl.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_hid_uart)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE
  ${app_sources}
  src/ancs_c.c
  src/i2c_eeprom.c
  src/sc7u22_spi.c
  src/gyroscope.c
  )

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
