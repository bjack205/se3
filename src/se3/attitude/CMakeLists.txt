add_library(se3_attitude INTERFACE quaternion.hpp)
target_include_directories(se3_attitude INTERFACE ${PROJECT_SOURCE_DIR}/src)
add_library(se3::attitude ALIAS se3_attitude)

# print include directories for se3::attitude
include(CMakePrintHelpers)
cmake_print_properties(TARGETS se3::attitude PROPERTIES INTERFACE_INCLUDE_DIRECTORIES)

