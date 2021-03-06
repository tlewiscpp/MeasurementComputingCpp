cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
project (MeasurementComputingCpp CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(mccusb-root)
set (MCCUSB_LIBUSB_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mccusb-root/Linux_Drivers/USB/")
set (SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src")
set (INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set (HEADER_ROOT "${INCLUDE_ROOT}/MeasurementComputingCpp")

file(GLOB ${PROJECT_NAME}_GLOB_SOURCE_FILES "${SOURCE_ROOT}/*.cpp")
file(GLOB ${PROJECT_NAME}_GLOB_HEADER_FILES "${HEADER_ROOT}/*.hpp")

set (${PROJECT_NAME}_SOURCE_FILES
     ${${PROJECT_NAME}_GLOB_SOURCE_FILES})

set (${PROJECT_NAME}_HEADER_FILES
     ${${PROJECT_NAME}_GLOB_HEADER_FILES})

add_library(${PROJECT_NAME} SHARED
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})

add_library(${PROJECT_NAME}_STATIC STATIC
            ${${PROJECT_NAME}_SOURCE_FILES}
            ${${PROJECT_NAME}_HEADER_FILES})

set_target_properties(${PROJECT_NAME}_STATIC PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        usb-1.0
        hidapi-libusb
        mccusb)

target_link_libraries(${PROJECT_NAME}_STATIC
                      usb-1.0
                      hidapi-libusb
                      mccusb)

target_include_directories(${PROJECT_NAME}
                           PUBLIC "${CMAKE_SOURCE_DIR}"
                           PUBLIC "${SOURCE_ROOT}"
                           PUBLIC "${INCLUDE_ROOT}"
                           PUBLIC "${HEADER_ROOT}"
                           PUBLIC "${MCCUSB_LIBUSB_SOURCE_DIR}")

target_include_directories(${PROJECT_NAME}_STATIC
                           PUBLIC "${CMAKE_SOURCE_DIR}"
                           PUBLIC "${SOURCE_ROOT}"
                           PUBLIC "${INCLUDE_ROOT}"
                           PUBLIC "${HEADER_ROOT}"
                           PUBLIC "${MCCUSB_LIBUSB_SOURCE_DIR}")



if(${CMAKE_SYSTEM_NAME} MATCHES Linux|.*BSD|DragonFly)

    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${${PROJECT_NAME}_HEADER_FILES}")

    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/")

endif()
