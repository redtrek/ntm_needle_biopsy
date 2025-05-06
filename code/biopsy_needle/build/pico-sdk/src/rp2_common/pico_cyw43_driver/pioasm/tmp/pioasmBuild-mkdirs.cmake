# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/libs/pico-sdk/tools/pioasm"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pioasm"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pioasm-install"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
