cmake_minimum_required(VERSION 3.8)
project(icm20948_i2cdev)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Define DMP usage
add_compile_definitions(ICM_20948_USE_DMP)

# Create the ICM20948 I2C library
add_library(icm20948_i2cdev
  src/ICM_20948.cpp
  src/i2c_wrapper.cpp
  src/util/ICM_20948_C.c
)

# Include directories for the library
target_include_directories(icm20948_i2cdev
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/util>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Set C standard for C source files
set_source_files_properties(
  src/util/ICM_20948_C.c
  PROPERTIES
  LANGUAGE C
  C_STANDARD 99
)

# Expose public headers
target_compile_features(icm20948_i2cdev PUBLIC cxx_std_17)

# Export targets
install(TARGETS icm20948_i2cdev
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY src/
  DESTINATION include/icm20948_i2cdev
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Build test example
add_executable(test examples/PortableC/test.cpp)
target_link_libraries(test PRIVATE icm20948_i2cdev)
