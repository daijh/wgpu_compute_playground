set(target matmulnbits)

# cxxopts
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG v3.2.1
  EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(cxxopts)
include_directories("${cxxopts_SOURCE_DIR}/include")

set(LINK_LIB ${LINK_LIB} dawn::dawn_native)
set(LINK_LIB ${LINK_LIB} dawn::dawn_proc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

aux_source_directory(. SRCS)
aux_source_directory(../common SRCS_COMMON)

add_executable(${target} ${SRCS} ${SRCS_COMMON})
target_link_libraries(${target} ${LINK_LIB})
