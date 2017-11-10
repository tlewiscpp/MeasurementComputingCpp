cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME MeasurementComputingCpp)
project (${PROJECT_NAME} CXX)

set(MCCUSB_LIBUSB_SOURCE_DIR ../mccusb-root/Linux_Drivers/USB/)

set (SOURCE_ROOT .)

set (${PROJECT_NAME}_SOURCE_FILES
        ${SOURCE_ROOT}/USB_1024LS.cpp
        ${SOURCE_ROOT}/USB_1208LS.cpp
        ${SOURCE_ROOT}/USB_1208FS.cpp
        ${SOURCE_ROOT}/USB_1608FS.cpp)

set (${PROJECT_NAME}_HEADER_FILES
        ${SOURCE_ROOT}/USB_1024LS.hpp
        ${SOURCE_ROOT}/USB_1208LS.hpp
        ${SOURCE_ROOT}/USB_1208FS.hpp
        ${SOURCE_ROOT}/USB_1608FS.hpp)

if (SHARED_LIB_BUILD)
    set(BUILD_TYPE "SHARED"
else()
    set(BUILD_TYPE "STATIC")
endif()

add_library(${PROJECT_NAME} ${BUILD_TYPE}
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
        usb-1.0-static
        hidapi-libusb-static
        mccusb-libusb-static)

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
        PUBLIC ${SOURCE_ROOT}/
        PUBLIC ${MCCUSB_LIBUSB_SOURCE_DIR}/)

