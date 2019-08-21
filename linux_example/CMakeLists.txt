cmake_minimum_required(VERSION 2.4.6)

project (LpmsIG1SimpleExample)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE "Release")
set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

set(IG1_INC "./..")
set(IG1_LIB "./lib")

include_directories("${IG1_INC}")
link_directories("${IG1_LIB}")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

set(sources
    ${IG1_INC}/LpMatrix.c
    main.cpp
)
set(headers
    ${IG1_INC}/LpMatrix.h
    ${IG1_INC}/LpmsIG1Registers.h
    ${IG1_INC}/LpmsIG1I.h
    ${IG1_INC}/SensorDataI.h
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11") 
endif()

add_executable(${CMAKE_PROJECT_NAME}  ${sources} ${headers})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_libraries(${CMAKE_PROJECT_NAME} LpmsIG1_OpenSourceLib)
    target_link_libraries(${CMAKE_PROJECT_NAME} rt)
    target_link_libraries(${CMAKE_PROJECT_NAME} pthread)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(${CMAKE_PROJECT_NAME} LpmsIG1_OpenSourceLibD)
    target_link_libraries(${CMAKE_PROJECT_NAME} rt)
    target_link_libraries(${CMAKE_PROJECT_NAME} pthread)
endif()


