cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME hidapi-libusb-static)
project (${PROJECT_NAME})
set (BUILD_SHARED_LIBRARIES FALSE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SOURCE_ROOT hidapi/hidapi)
set(LIBUSB_ROOT ../libusb-root/libusb/libusb)

set (${PROJECT_NAME}_SOURCE_FILES ${SOURCE_ROOT}/../libusb/hid.c)
set (${PROJECT_NAME}_HEADER_FILES ${SOURCE_ROOT}/hidapi.h)

add_library(${PROJECT_NAME} STATIC
        ${${PROJECT_NAME}_SOURCE_FILES}
        ${${PROJECT_NAME}_HEADER_FILES})

if (WIN32)
    target_link_libraries(${PROJECT_NAME} usb-1.0-static -liconv)
elseif(ANDROID)
    target_link_libraries(${PROJECT_NAME} usb-1.0-static)
else()
    target_link_libraries(${PROJECT_NAME} usb-1.0-static)
endif()

if (MINGW)
    target_include_directories(${PROJECT_NAME}
        PUBLIC ${SOURCE_ROOT}/
        PUBLIC ${LIBUSB_ROOT}/)
else()
target_include_directories(${PROJECT_NAME}
        PUBLIC ${SOURCE_ROOT}/
        PUBLIC ${LIBUSB_ROOT}/)
endif()