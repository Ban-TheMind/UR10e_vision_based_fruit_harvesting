cmake_minimum_required(VERSION 3.16)
project(robotiq_sdk_bridge LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(GRIPPERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GRIPPERS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GRIPPERS_BUILD_FAKE OFF CACHE BOOL "" FORCE)
set(GRIPPERS_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
# Build the SDK's transport dependency from an unmodified pinned release.
include(ExternalProject)
find_program(SERIALPORT_MAKE make)
if(NOT SERIALPORT_MAKE)
  message(FATAL_ERROR "make is required to build the official SDK serial transport")
endif()
set(SERIALPORT_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/serialport")
file(MAKE_DIRECTORY "${SERIALPORT_PREFIX}/include")
ExternalProject_Add(serialport_build
  URL "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libserialport/libserialport-0.1.2.tar.gz"
  URL_HASH SHA256=5deb92b5ca72c0347b07b786848350deca2dcfd975ce613b8e0e1d947a4b4ca9
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=${SERIALPORT_PREFIX}
    --disable-shared --enable-static
    "CC=${CMAKE_C_COMPILER}" "CFLAGS=-O2 -fPIC"
  BUILD_COMMAND ${SERIALPORT_MAKE}
  INSTALL_COMMAND ${SERIALPORT_MAKE} install
  BUILD_BYPRODUCTS "${SERIALPORT_PREFIX}/lib/libserialport.a")
add_library(serialport::serialport STATIC IMPORTED GLOBAL)
set_target_properties(serialport::serialport PROPERTIES
  IMPORTED_LOCATION "${SERIALPORT_PREFIX}/lib/libserialport.a"
  INTERFACE_INCLUDE_DIRECTORIES "${SERIALPORT_PREFIX}/include")
add_dependencies(serialport::serialport serialport_build)
add_subdirectory(vendor/robotiq/sdk_cpp EXCLUDE_FROM_ALL)
add_executable(robotiq_sdk_command src/main.cpp)
target_link_libraries(robotiq_sdk_command PRIVATE Robotiq::grippers)
install(TARGETS robotiq_sdk_command DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_bridge test/test_bridge.cpp)
  target_include_directories(test_bridge PRIVATE src vendor/robotiq/sdk_cpp/tests)
  target_link_libraries(test_bridge PRIVATE Robotiq::grippers)
  add_test(NAME robotiq_bridge_policy COMMAND test_bridge)
endif()
ament_package()
