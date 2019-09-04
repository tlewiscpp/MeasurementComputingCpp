cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
project (mccusb C)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set (SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Linux_Drivers/USB/mcc-libusb/")
set (INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Linux_Drivers/USB/mcc-libusb/")
set (HEADER_ROOT "${INCLUDE_ROOT}")

file(GLOB ${PROJECT_NAME}_GLOB_SOURCE_FILES "${SOURCE_ROOT}/*.c")
file(GLOB ${PROJECT_NAME}_GLOB_HEADER_FILES "${HEADER_ROOT}/*.h")

list(FILTER ${PROJECT_NAME}_GLOB_SOURCE_FILES EXCLUDE REGEX "test-*")

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
                      hidapi-libusb)

target_link_libraries(${PROJECT_NAME}_STATIC
                      usb-1.0
                      hidapi-libusb)


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


    set (UDEV_FILE_LOCATION "Linux_Drivers")

    install(FILES "${UDEV_FILE_LOCATION}/61-mcc.rules" DESTINATION "/etc/udev/rules.d")


endif()



