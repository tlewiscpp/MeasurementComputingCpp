cmake_minimum_required (VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set (PROJECT_NAME mccusb)
project (${PROJECT_NAME})
set (BUILD_SHARED_LIBRARIES FALSE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set (SOURCE_ROOT Linux_Drivers/USB/mcc-libusb/)

set (${PROJECT_NAME}_SOURCE_FILES
        ${SOURCE_ROOT}/pmd.c
        ${SOURCE_ROOT}/usb-1608G.c
        ${SOURCE_ROOT}/usb-20X.c
        ${SOURCE_ROOT}/usb-1208FS-Plus.c
        ${SOURCE_ROOT}/usb-1608FS-Plus.c
        ${SOURCE_ROOT}/usb-2020.c
        ${SOURCE_ROOT}/usb-ctr.c
        ${SOURCE_ROOT}/usb-2600.c
        ${SOURCE_ROOT}/usb-2408.c
        ${SOURCE_ROOT}/usb-2416.c
        ${SOURCE_ROOT}/usb-1608HS.c
        ${SOURCE_ROOT}/usb-1208HS.c
        ${SOURCE_ROOT}/usb-1024LS.c
        ${SOURCE_ROOT}/usb-1208LS.c
        ${SOURCE_ROOT}/usb-1608FS.c
        ${SOURCE_ROOT}/usb-7202.c
        ${SOURCE_ROOT}/usb-tc.c
        ${SOURCE_ROOT}/usb-dio24.c
        ${SOURCE_ROOT}/usb-dio96H.c
        ${SOURCE_ROOT}/usb-5200.c
        ${SOURCE_ROOT}/usb-temp.c
        ${SOURCE_ROOT}/usb-7204.c
        ${SOURCE_ROOT}/usb-1208FS.c
        ${SOURCE_ROOT}/usb-ssr.c
        ${SOURCE_ROOT}/usb-erb.c
        ${SOURCE_ROOT}/usb-pdiso8.c
        ${SOURCE_ROOT}/usb-1408FS.c
        ${SOURCE_ROOT}/usb-1616FS.c
        ${SOURCE_ROOT}/usb-3100.c
        ${SOURCE_ROOT}/usb-4303.c
        ${SOURCE_ROOT}/usb-tc-ai.c
        ${SOURCE_ROOT}/usb-dio32HS.c
        ${SOURCE_ROOT}/usb-tc-32.c
        ${SOURCE_ROOT}/bth-1208LS.c
        ${SOURCE_ROOT}/minilab-1008.c
        ${SOURCE_ROOT}/usb-1808.c
        ${SOURCE_ROOT}/nist.c)

set (${PROJECT_NAME}_HEADER_FILES
        ${SOURCE_ROOT}/pmd.h
        ${SOURCE_ROOT}/usb-1608G.h
        ${SOURCE_ROOT}/usb-20X.h
        ${SOURCE_ROOT}/usb-1208FS-Plus.h
        ${SOURCE_ROOT}/usb-1608FS-Plus.h
        ${SOURCE_ROOT}/usb-2020.h
        ${SOURCE_ROOT}/usb-ctr.h
        ${SOURCE_ROOT}/usb-2600.h
        ${SOURCE_ROOT}/usb-2408.h
        ${SOURCE_ROOT}/usb-2416.h
        ${SOURCE_ROOT}/usb-1608HS.h
        ${SOURCE_ROOT}/usb-1208HS.h
        ${SOURCE_ROOT}/usb-1024LS.h
        ${SOURCE_ROOT}/usb-1208LS.h
        ${SOURCE_ROOT}/usb-1608FS.h
        ${SOURCE_ROOT}/usb-7202.h
        ${SOURCE_ROOT}/usb-tc.h
        ${SOURCE_ROOT}/usb-dio24.h
        ${SOURCE_ROOT}/usb-dio96H.h
        ${SOURCE_ROOT}/usb-5200.h
        ${SOURCE_ROOT}/usb-temp.h
        ${SOURCE_ROOT}/usb-7204.h
        ${SOURCE_ROOT}/usb-1208FS.h
        ${SOURCE_ROOT}/usb-ssr.h
        ${SOURCE_ROOT}/usb-erb.h
        ${SOURCE_ROOT}/usb-1408FS.h
        ${SOURCE_ROOT}/usb-pdiso8.h
        ${SOURCE_ROOT}/usb-1616FS.h
        ${SOURCE_ROOT}/usb-3100.h
        ${SOURCE_ROOT}/usb-4303.h
        ${SOURCE_ROOT}/usb-tc-ai.h
        ${SOURCE_ROOT}/usb-dio32HS.h
        ${SOURCE_ROOT}/usb-tc-32.h
        ${SOURCE_ROOT}/bth-1208LS.h
        ${SOURCE_ROOT}/minilab-1008.h
        ${SOURCE_ROOT}/usb-1808.h
        ${SOURCE_ROOT}/usb-500.h)


add_library(${PROJECT_NAME} STATIC 
    ${${PROJECT_NAME}_SOURCE_FILES}
    ${${PROJECT_NAME}_HEADER_FILES})


target_link_libraries(${PROJECT_NAME}
        usb-1.0
        hidapi-libusb)

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/
        PUBLIC ${SOURCE_ROOT}/)

if(${CMAKE_SYSTEM_NAME} MATCHES Linux|.*BSD|DragonFly)

    set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${${PROJECT_NAME}_HEADER_FILES}")

    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/"
            PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/")


    set (UDEV_FILE_LOCATION ${SOURCE_ROOT}/mccusb-root/)

    install(FILES "${UDEV_FILE_LOCATION}/61-mcc.rules" DESTINATION "/etc/udev/rules.d")


endif()



