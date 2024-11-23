# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/chris/esp/v4.4.2/esp-idf/components/bootloader/subproject"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix/tmp"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix/src"
  "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Programming/playground/ESP-HAR/model_deployment/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
