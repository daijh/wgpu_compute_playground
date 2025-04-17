cmake_minimum_required(VERSION 3.16)
project(wgpu_compute_playground)

include(FetchContent)

# dawn
include(config/dawn.cmake)
add_subdirectory("dawn" EXCLUDE_FROM_ALL)

# fp16
FetchContent_Declare(
  fp16
  GIT_REPOSITORY https://github.com/Maratyszcza/FP16.git
  GIT_TAG b3720617faf1a4581ed7e6787cc51722ec7751f0
  EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(fp16)
include_directories("${fp16_SOURCE_DIR}/include")

# config
include(config/compiler.cmake)
include(config/format.cmake)

# src
add_subdirectory(src)
