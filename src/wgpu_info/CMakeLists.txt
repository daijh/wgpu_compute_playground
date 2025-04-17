set(target wgpu_info)

set(LINK_LIB ${LINK_LIB} dawn::dawn_native)
set(LINK_LIB ${LINK_LIB} dawn::dawn_proc)

aux_source_directory(. SRCS)

add_executable(${target} ${SRCS})
target_link_libraries(${target} ${LINK_LIB})
