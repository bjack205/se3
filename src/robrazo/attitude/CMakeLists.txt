add_library(robrazo_attitude
    quaternion.cpp
)
target_include_directories(robrazo_attitude
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
