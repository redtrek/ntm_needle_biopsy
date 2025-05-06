# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/_deps/picotool-src"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/_deps/picotool-build"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/_deps"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/tmp"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/src/picotoolBuild-stamp"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/src"
  "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/redtrek/NTM/ntm_needle_biopsy/code/biopsy_needle/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
