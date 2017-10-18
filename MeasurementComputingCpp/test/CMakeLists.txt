PROJECT (${PROJECT_NAME} CXX)
cmake_minimum_required(VERSION 3.1)
set (PROJECT_NAME USB_1608FS_Test)
set (CMAKE_CXX_STANDARD 14)

add_subdirectory(hidapi-root/)
add_subdirectory(libusb-root/)
add_subdirectory(MeasurementComputing/)

set (SOURCE_ROOT .)

set (${PROJECT_NAME}_SOURCE_FILES
    ${SOURCE_ROOT}/Main.cpp)

set (${PROJECT_NAME}_HEADER_FILES)

add_executable(${PROJECT_NAME} 
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
    usb-1.0-static
    hidapi-libusb-static
    MeasurementComputingCpp
        -lpthread)
