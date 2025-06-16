CPMAddPackage(
    NAME googletest
    GITHUB_REPOSITORY google/googletest
    VERSION 1.13.0
    OPTIONS
    "INSTALL_GTEST OFF"
    "gtest_force_shared_crt ON"
)
add_library(gtest::gtest ALIAS gtest_main)
add_library(gtest::gmock ALIAS gmock_main)
include(GoogleTest)

include(CMakeParseArguments)
function(add_se3_test NAME)
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs "SOURCES;LIBRARIES")
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${NAME} ${ARG_SOURCES})
    target_link_libraries(${NAME} PRIVATE gtest::gtest ${ARG_LIBRARIES})
    # target_include_directories(${NAME} PRIVATE ${PROJECT_SOURCE_DIR}/src)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_se3_test(quaternion_test
    SOURCES
        quaternion_test.cpp
    LIBRARIES
        se3::attitude
)
