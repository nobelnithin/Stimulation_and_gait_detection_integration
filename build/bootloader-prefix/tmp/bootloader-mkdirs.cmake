# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Admin/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/tmp"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/src"
  "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Biostim/Stimulation_demos/Stimulation_Check/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
