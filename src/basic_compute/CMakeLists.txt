set(target basic_compute)

set(LINK_LIB ${LINK_LIB} dawn::dawn_native)
set(LINK_LIB ${LINK_LIB} dawn::dawn_proc)
set(LINK_LIB ${LINK_LIB} wgpu_playground_common)

aux_source_directory(. SRCS)

add_executable(${target} ${SRCS} ${SRCS_COMMON})
target_link_libraries(${target} ${LINK_LIB})
