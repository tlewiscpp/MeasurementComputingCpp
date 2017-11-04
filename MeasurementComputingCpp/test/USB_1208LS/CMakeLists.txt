PROJECT (${PROJECT_NAME} CXX)
cmake_minimum_required(VERSION 3.1)
set (PROJECT_NAME USB_1208FS_Test)
set (CMAKE_CXX_STANDARD 14)


set(MCC_CPP_SOURCE_ROOT ../../)
set (SOURCE_ROOT .)

set (${PROJECT_NAME}_SOURCE_FILES
    ${SOURCE_ROOT}/Main.cpp)

set (${PROJECT_NAME}_HEADER_FILES)

add_executable(${PROJECT_NAME} 
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})

target_link_libraries(${PROJECT_NAME}
    -lMeasurementComputingCpp
    -lpthread)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${MCC_CPP_SOURCE_ROOT})
