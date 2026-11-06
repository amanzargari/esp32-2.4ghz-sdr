cmake_minimum_required(VERSION 3.16)

# Only build what main depends on. esp_phy pulls esp_wifi into the build graph,
# but its code is only linked if referenced, and nothing here starts the driver.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_sdr_firmware)
