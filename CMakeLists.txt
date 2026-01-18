cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32FMRadio)

# Create SPIFFS image with webui.html
spiffs_create_partition_image(spiffs ${CMAKE_CURRENT_SOURCE_DIR}/main FLASH_IN_PROJECT)
