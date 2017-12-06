cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME usb-1.0-static)
project (${PROJECT_NAME})
set (BUILD_SHARED_LIBRARIES FALSE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SOURCE_ROOT libusb/libusb)

if (WIN32)
set (${PROJECT_NAME}_SOURCE_FILES
    ${SOURCE_ROOT}/core.c
    ${SOURCE_ROOT}/descriptor.c
    ${SOURCE_ROOT}/hotplug.c
    ${SOURCE_ROOT}/io.c
    ${SOURCE_ROOT}/sync.c
    ${SOURCE_ROOT}/strerror.c
    ${SOURCE_ROOT}/os/poll_windows.c
    ${SOURCE_ROOT}/os/windows_winusb.c
    ${SOURCE_ROOT}/os/windows_nt_common.c
    ${SOURCE_ROOT}/os/threads_windows.c)

set (${PROJECT_NAME}_HEADER_FILES
    ${SOURCE_ROOT}/hotplug.h
    ${SOURCE_ROOT}/os/poll_windows.h
    ${SOURCE_ROOT}/os/windows_winusb.h
    ${SOURCE_ROOT}/os/windows_nt_common.h
    ${SOURCE_ROOT}/os/threads_windows.h
    ${SOURCE_ROOT}/os/windows_common.h
    ${SOURCE_ROOT}/version.h
    ${SOURCE_ROOT}/version_nano.h
    ${SOURCE_ROOT}/libusbi.h
    ${SOURCE_ROOT}/libusb.h
        config.h)
else()
    set (${PROJECT_NAME}_SOURCE_FILES
            ${SOURCE_ROOT}/core.c
            ${SOURCE_ROOT}/descriptor.c
            ${SOURCE_ROOT}/hotplug.c
            ${SOURCE_ROOT}/io.c
            ${SOURCE_ROOT}/sync.c
            ${SOURCE_ROOT}/strerror.c
            ${SOURCE_ROOT}/os/linux_usbfs.c
            ${SOURCE_ROOT}/os/poll_posix.c
            ${SOURCE_ROOT}/os/threads_posix.c
            ${SOURCE_ROOT}/os/linux_netlink.c)

    set (${PROJECT_NAME}_HEADER_FILES
            ${SOURCE_ROOT}/hotplug.h
            ${SOURCE_ROOT}/version.h
            ${SOURCE_ROOT}/version_nano.h
            ${SOURCE_ROOT}/libusbi.h
            ${SOURCE_ROOT}/libusb.h
            config.h)

endif()


add_library(${PROJECT_NAME} STATIC
        ${${PROJECT_NAME}_SOURCE_FILES}
        ${${PROJECT_NAME}_HEADER_FILES})

if (WIN32)
    target_include_directories(${PROJECT_NAME}
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
            PUBLIC ${SOURCE_ROOT}/
            PUBLIC ${SOURCE_ROOT}/os/
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../win32-fake/)
else()
    target_include_directories(${PROJECT_NAME}
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
            PUBLIC ${SOURCE_ROOT}/
            PUBLIC ${SOURCE_ROOT}/os/)
endif()


