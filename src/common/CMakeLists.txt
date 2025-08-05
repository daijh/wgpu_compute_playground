set(target wgpu_playground_common)

set(LINK_LIB ${LINK_LIB} dawn::dawn_native)
set(LINK_LIB ${LINK_LIB} dawn::dawn_proc)

aux_source_directory(. SRCS)

if(WIN32)
  add_library(${target} STATIC ${SRCS})
else()
  add_library(${target} SHARED ${SRCS})
endif()

target_link_libraries(${target} ${LINK_LIB})

if(WIN32)
  set_property(TARGET ${target} PROPERTY WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
