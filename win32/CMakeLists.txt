cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME MeasurementComputingWin32)
project (${PROJECT_NAME})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT STATIC_BUILD)
    set (SHARED_LIB_BUILD)
    set(LIBRARY_TYPE SHARED)
else()
    unset(SHARED_LIB_BUILD)
    set (LIBRARY_TYPE STATIC)
endif()

set (SOURCE_ROOT src/)
set (MCC_COMMON common/)

set (${PROJECT_NAME}_SOURCE_FILES)

set (${PROJECT_NAME}_HEADER_FILES
        ${MCC_COMMON}/cbw.h)

add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
        ${${PROJECT_NAME}_SOURCE_FILES}
        ${${PROJECT_NAME}_HEADER_FILES})

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${SOURCE_ROOT}/)

target_link_libraries(${PROJECT_NAME} ${MCC_COMMON}/cb64.lib)

#if(${CMAKE_SYSTEM_NAME} MATCHES Linux|.*BSD|DragonFly)
#
#    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${${PROJECT_NAME}_HEADER_FILES}")
#
#    install(TARGETS ${PROJECT_NAME}
#            ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
#            LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
#            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/")
#
#endif()


