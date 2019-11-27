#***********************************************************************
#* Copyright (C) 2019 LP-Research
#* All rights reserved.
#* Contact: LP-Research (info@lp-research.com)
#*
#* This file is part of the Open Motion Analysis Toolkit (OpenMAT).
#*
#* Redistribution and use in source and binary forms, with 
#* or without modification, are permitted provided that the 
#* following conditions are met:
#*
#* Redistributions of source code must retain the above copyright 
#* notice, this list of conditions and the following disclaimer.
#* Redistributions in binary form must reproduce the above copyright 
#* notice, this list of conditions and the following disclaimer in 
#* the documentation and/or other materials provided with the 
#* distribution.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#**********************************************************************/

cmake_minimum_required(VERSION 2.4.6)


set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release" )

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
#set(BUILD_ARCHITECTURE "64-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    project (LpmsIG1_OpenSourceLib)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    project (LpmsIG1_OpenSourceLibD)
endif()

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
    cmake_policy(SET CMP0040 NEW)
endif(COMMAND cmake_policy)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Windows SDK
    set(WINDOWS_SDK_PATH "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A" CACHE STRING "")
    
    # LpmsIG1
    include_directories("./")

    ADD_DEFINITIONS(-DUSE_EIGEN)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
    ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
    ADD_DEFINITIONS(-DPLATFORM_X86)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ADD_DEFINITIONS(-DDLL_EXPORT)

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
    endif()
    
    #link_directories("${WINDOWS_SDK_PATH}/Lib")
endif()


if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(sources
        MicroMeasureWindows.cpp
        SerialPort.cpp
        LpmsIG1.cpp
        LpMatrix.c
        util.cpp
        version.rc
    )
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(sources
        MicroMeasure.cpp
        SerialPortLinux.cpp
        LpmsIG1.cpp
        LpMatrix.c
        util.cpp
        version.rc
    )
    set(headers_linux_install
        LpMatrix.h
        LpmsIG1Registers.h
        SensorDataI.h
        LpmsIG1I.h
    )
endif()

  
set(headers
    MicroMeasure.h
    SerialPort.h
    SensorData.h
    SensorDataI.h
    util.h
    LpmsIG1.h
    LpmsIG1I.h
    LpmsIG1Registers.h
    LpMatrix.h
)


# Silicon labs
set(WINDOWS_SL_PATH "E:/openSourceLibSiliconLabs/MCU/USBXpress_SDK/Library/Host/Windows" CACHE STRING "")
include_directories("${WINDOWS_SL_PATH}")
link_directories("${WINDOWS_SL_PATH}/x86")
find_library(SILABS_LIB SiUSBXp.lib PATHS "${WINDOWS_SL_PATH}/x86")

if (BUILD_ARCHITECTURE STREQUAL "32-bit")
    set(LIBRARY_OUTPUT_PATH ./ CACHE STRING "")
    set(EXECUTABLE_OUTPUT_PATH ./ CACHE STRING "")
endif()

if (BUILD_ARCHITECTURE STREQUAL "64-bit")
    set(LIBRARY_OUTPUT_PATH ./ CACHE STRING "")
    set(EXECUTABLE_OUTPUT_PATH ./ CACHE STRING "")
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11") 
    #SET ( CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=1" )
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if (BUILD_ARCHITECTURE STREQUAL "32-bit")
        if(CMAKE_BUILD_TYPE STREQUAL "Release")
            add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
            target_link_libraries(${CMAKE_PROJECT_NAME} Ws2_32.lib)
            target_link_libraries(${CMAKE_PROJECT_NAME} SiUSBXp.lib)
        endif()

        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
            target_link_libraries(${CMAKE_PROJECT_NAME} Ws2_32.lib)
            target_link_libraries(${CMAKE_PROJECT_NAME} SiUSBXp.lib)
        endif()
    endif()

    if (BUILD_ARCHITECTURE STREQUAL "64-bit")
        if(CMAKE_BUILD_TYPE STREQUAL "Release")
            add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
            target_link_libraries(${CMAKE_PROJECT_NAME} Ws2_32.lib)
            target_link_libraries(${CMAKE_PROJECT_NAME} SiUSBXp.lib)
        endif()

        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
            target_link_libraries(${CMAKE_PROJECT_NAME} Ws2_32.lib)
            target_link_libraries(${CMAKE_PROJECT_NAME} SiUSBXp.lib)
        endif()
    endif()
    
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    include(InstallRequiredSystemLibraries)
    set(MAJOR_VERSION "0")
    set(MINOR_VERSION "2")
    set(PATCH_VERSION "2")
    SET(CPACK_GENERATOR "DEB")
    SET(CPACK_PACKAGE_NAME "libLpmsIG1_OpenSource")
    set(CPACK_PACKAGE_VENDOR "LP-Research Inc. <www.lp-research.com>")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library for communicating and interfacing with LP-Research sensors.")
    set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/SUMMARY.txt")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
    set(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
    set(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
    set(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
    SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "H.E. YAP <yap@lp-research.com>") #required
    
    include(CPack)

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
        target_link_libraries(${CMAKE_PROJECT_NAME} pthread)
        target_link_libraries(${CMAKE_PROJECT_NAME} rt)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_library(${CMAKE_PROJECT_NAME} SHARED ${sources} ${headers})
        target_link_libraries(${CMAKE_PROJECT_NAME} pthread)
        target_link_libraries(${CMAKE_PROJECT_NAME} rt)
    endif()

    install(FILES ${headers_linux_install} DESTINATION include/lpsensor)

    install(TARGETS ${CMAKE_PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()
