set(target basic_compute)

set(LINK_LIB ${LINK_LIB} dawn::dawn_native)
set(LINK_LIB ${LINK_LIB} dawn::dawn_proc)

aux_source_directory(. SRCS)
aux_source_directory(../common SRCS_COMMON)

add_executable(${target} ${SRCS} ${SRCS_COMMON})
target_link_libraries(${target} ${LINK_LIB})
