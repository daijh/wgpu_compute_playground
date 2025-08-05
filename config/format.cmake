if("Linux" STREQUAL ${CMAKE_HOST_SYSTEM_NAME})
  add_custom_target(
    format
    COMMAND
      bash -c
      " \
    find ${CMAKE_CURRENT_SOURCE_DIR}/src \
    -regextype posix-extended -regex '.*\\.(c|cc|cpp|h|hpp)$' \
    | xargs -I '{}' /usr/bin/clang-format --verbose -i '{}' && \
\
    cmake-format -i ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/config/* && \
\
    find ${CMAKE_CURRENT_SOURCE_DIR}/src \
    -name CMakeLists.txt \
    | xargs -I '{}' cmake-format -i '{}' && \
\
    markdownlint-cli2 --config ${CMAKE_CURRENT_SOURCE_DIR}/.markdownlint.yaml --fix ${CMAKE_CURRENT_SOURCE_DIR}/README.md && \
\
    find ${CMAKE_CURRENT_SOURCE_DIR}/src \
    -regextype posix-extended -regex '.*\\.(md)$' \
    | xargs -I '{}' markdownlint-cli2 --config ${CMAKE_CURRENT_SOURCE_DIR}/.markdownlint.yaml --fix '{}' \
    "
    VERBATIM)
endif()
