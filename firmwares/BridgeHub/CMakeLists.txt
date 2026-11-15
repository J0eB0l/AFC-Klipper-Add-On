# BridgeHub firmware for an ESP32-S3 board (ESP-IDF 5.3):
#
#   . $IDF_PATH/export.sh
#   idf.py build                 # -> build/bridgehub.bin
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bridgehub)
