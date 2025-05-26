# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(DTC_OVERLAY_FILE "~/csse4011/project/sisyphus-red/mycode/apps/ultrasonic/boards/nrf52840dk_nrf52840.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ultrasonic)

target_sources(app PRIVATE src/main.c)
