# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "H:/Users/Administrator/.espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/tmp"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/src"
  "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Mehrshad_Projects/ESP_IDF/workspace/ads129x_driver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
