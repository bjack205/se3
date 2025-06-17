add_library(se3_attitude INTERFACE quaternion.hpp quaternion_ops.hpp)
target_link_libraries(se3_attitude INTERFACE se3::linalg)
target_include_directories(se3_attitude INTERFACE ${PROJECT_SOURCE_DIR}/src)
target_compile_features(se3_attitude INTERFACE cxx_std_20)
add_library(se3::attitude ALIAS se3_attitude)

# print include directories for se3::attitude
include(CMakePrintHelpers)
cmake_print_properties(TARGETS se3::attitude PROPERTIES INTERFACE_INCLUDE_DIRECTORIES)

