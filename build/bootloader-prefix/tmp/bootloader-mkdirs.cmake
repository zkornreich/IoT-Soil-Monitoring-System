# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/zackk/esp/v5.3.1/esp-idf/components/bootloader/subproject"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/tmp"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/src"
  "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Binghamton/Masters_Year/IoT/Round2/tutorial/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
