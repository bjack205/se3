cmake_minimum_required(VERSION 3.20)

list(APPEND CMAKE_MESSAGE_CONTEXT se3)
project(se3
    VERSION 0.1.0
    LANGUAGES C CXX
)
set(PREFIX SE3)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakePrintHelpers)
include(FindCPM)

# Use Release build type by default if not specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Options
option(${PREFIX}_BUILD_TEST ON)

# Find Python
if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()
find_package(Python 3.10 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED 
  OPTIONAL_COMPONENTS Development.SABIModule)

# Add nanobind
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# Enable AVX and AVX2 for SIMD code
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  add_compile_options(-mavx -mavx2)
elseif (MSVC)
  add_compile_options(/arch:AVX2)
endif()

# Dependencies
# TODO: make this optional
CPMAddPackage(
    NAME Eigen
    VERSION 3.4.0
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    # Eigen's CMakelists are not intended for library use
    DOWNLOAD_ONLY YES
)

if(Eigen_ADDED)
  add_library(Eigen INTERFACE IMPORTED
      test/vector_tests.cpp)
  target_include_directories(Eigen INTERFACE ${Eigen_SOURCE_DIR})
endif()

# Build
add_subdirectory(src/se3)

# Tests
if(${PREFIX}_BUILD_TEST)
    enable_testing()
    include(CTest)
    add_subdirectory(test)
endif()

# Install
install(TARGETS se3_ext LIBRARY DESTINATION se3)
