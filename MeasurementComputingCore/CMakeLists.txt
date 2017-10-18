cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME MeasurementComputingCpp)
project (${PROJECT_NAME})
set(BUILD_SHARED_LIBS FALSE)

add_subdirectory(mccusb-root/)
