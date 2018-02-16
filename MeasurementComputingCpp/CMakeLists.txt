cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME MeasurementComputingCpp)
project (${PROJECT_NAME} CXX)

if (WIN32 OR WIN64)
    set(IMPL_DIR win32)
    set(WINDOWS_COMMON win32/common/)
else()
    set(IMPL_DIR posix)
    set(MCCUSB_LIBUSB_SOURCE_DIR ${IMPL_DIR}/mccusb-root/Linux_Drivers/USB/)
    add_subdirectory(${IMPL_DIR}/mccusb-root)
endif()


set (SOURCE_ROOT .)

set (${PROJECT_NAME}_SOURCE_FILES
        ${IMPL_DIR}/USB_IO_Base.cpp
        ${IMPL_DIR}/USB_1024LS.cpp
        ${IMPL_DIR}/USB_1208LS.cpp
        ${IMPL_DIR}/USB_1208FS.cpp
        ${IMPL_DIR}/USB_1608FS.cpp)

set (${PROJECT_NAME}_HEADER_FILES
        ${SOURCE_ROOT}/USB_IO_Base.hpp
        ${SOURCE_ROOT}/USB_1024LS.hpp
        ${SOURCE_ROOT}/USB_1208LS.hpp
        ${SOURCE_ROOT}/USB_1208FS.hpp
        ${SOURCE_ROOT}/USB_1608FS.hpp)

if (SHARED_LIB_BUILD)
    set(BUILD_TYPE "SHARED")
else()
    set(BUILD_TYPE "STATIC")
endif()

add_library(${PROJECT_NAME} ${BUILD_TYPE}
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})

if (WIN32 OR WIN64)
    link_directories(${WINDOWS_COMMON})
    if(NOT "${CMAKE_GENERATOR}" MATCHES "(Win64|IA64)")
        target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/${WINDOWS_COMMON}/cbw32.lib)
    else()
        target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/${WINDOWS_COMMON}/cbw64.lib)
    endif()


    target_include_directories(${PROJECT_NAME}
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
            PUBLIC ${SOURCE_ROOT}/)
else()
    target_link_libraries(${PROJECT_NAME}
            usb-1.0
            hidapi-libusb
            mccusb)

    target_include_directories(${PROJECT_NAME}
            PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
            PUBLIC ${SOURCE_ROOT}/
            PUBLIC ${MCCUSB_LIBUSB_SOURCE_DIR}/)
endif()



if(${CMAKE_SYSTEM_NAME} MATCHES Linux|.*BSD|DragonFly)

    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${${PROJECT_NAME}_HEADER_FILES}")

    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/")

endif()
