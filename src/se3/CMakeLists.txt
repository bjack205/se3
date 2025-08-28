include_directories("${PROJECT_SOURCE_DIR}/src")
add_subdirectory(linear_algebra)
add_subdirectory(attitude)

add_library(se3_se3 INTERFACE
    generic.hpp)
target_link_libraries(se3_se3 INTERFACE se3::linalg se3::attitude)
add_library(se3::se3 ALIAS se3_se3)

nanobind_add_module(
  # Name of the extension
  se3_ext

  # Target the stable ABI for Python 3.12+, which reduces
  # the number of binary wheels that must be built. This
  # does nothing on older Python versions
  STABLE_ABI

  # Build libnanobind statically and merge it into the
  # extension (which itself remains a shared library)
  #
  # If your project builds multiple extensions, you can
  # replace this flag by NB_SHARED to conserve space by
  # reusing a shared libnanobind across libraries
  NB_STATIC

  # Source code goes here
  se3_pybind.cpp
)