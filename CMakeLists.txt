cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME MeasurementComputingCpp)
project (${PROJECT_NAME} CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set (HIDAPI_ROOT hidapi-root)
set (LIBUSB_ROOT libusb-root)
set (MCCUSB_ROOT mccusb-root)
set (MCCCPP_ROOT MeasurementComputingCpp)


add_subdirectory(${LIBUSB_ROOT})
add_subdirectory(${HIDAPI_ROOT})
add_subdirectory(${MCCUSB_ROOT})
add_subdirectory(${MCCCPP_ROOT})
