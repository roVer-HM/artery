project(Artery)
cmake_minimum_required(VERSION 3.1)
if(POLICY CMP0076)
    cmake_policy(SET CMP0076 NEW)
endif()
if(POLICY CMP0079)
    cmake_policy(SET CMP0079 OLD)
endif()
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)
enable_testing()
add_compile_options(-fPIC)

find_package(OmnetPP 5.4 REQUIRED)
include(cmake/ImportOppTarget.cmake)
include(cmake/GenerateOppMessage.cmake)
include(cmake/GetNedFolders.cmake)
include(cmake/AddOppBuildTarget.cmake)
include(cmake/AddOppRun.cmake)
include(ProcessorCount)

set(VANETZA_PATH  ${PROJECT_SOURCE_DIR}/extern/vanetza CACHE STRING "Path to dependency")
set(VEINS_PATH  ${PROJECT_SOURCE_DIR}/../veins CACHE STRING "Path to dependency default in extern/*")
set(INET_PATH ${PROJECT_SOURCE_DIR}/../inet4 CACHE STRING "Path to dependency")
set(SIMULTE_PATH ${PROJECT_SOURCE_DIR}/../simulte CACHE STRING "Path to dependency")

# Postifx
set(CMAKE_DEBUG_POSTFIX "_dbg")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/static)


if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_BUILD_TYPE Release)
    set(MAKE_MODE release)
else()
	set(CMAKE_BUILD_TYPE Debug)
	set(MAKE_MODE debug)
endif()

set(VANETZA_BINARY_DIR ${VANETZA_PATH}/build/${CMAKE_BUILD_TYPE})

find_package(Vanetza CONFIG PATHS ${VANETZA_BINARY_DIR} REQUIRED NO_CMAKE_PACKAGE_REGISTRY)

find_path(Veins_DIR NAMES src/veins/package.ned PATHS ${VEINS_PATH} DOC "Veins root directory")
import_opp_target(veins ${Veins_DIR}/src/Makefile)

find_path(INET_DIR NAMES src/inet/package.ned PATHS ${INET_PATH} DOC "INET root directory")
import_opp_target(INET ${INET_DIR}/src/Makefile)

option(WITH_SIMULTE "Build Artery with SimuLTE integration" OFF)
if(WITH_SIMULTE)
    message(SEND_ERROR "SimuLTE is not yet compatible with INET4. Please turn off WITH_SIMULTE.")
    find_path(SimuLTE_DIR NAMES src/package.ned PATHS ${SIMULTE_PATH} DOC "SimuLTE root directory")
    import_opp_target(lte ${SimuLTE_DIR}/src/Makefile)
else()
    message(STATUS "SimuLTE integration disabled")
endif()

find_program(MAKE_COMMAND NAMES make gmake HINTS ${CMAKE_MAKE_COMMAND} DOC "Makefile processor")
ProcessorCount(N)
if(NOT N EQUAL 0)
    set(MAKE_ARGUMENTS "-j${N};MODE=${MAKE_MODE}" CACHE STRING "Additional arguments passed to Makefile processor")
else()
    set(MAKE_ARGUMENTS ";MODE=${MAKE_MODE}" CACHE STRING "Additional arguments passed to Makefile processor")
endif()

string(REPLACE " " ";" MAKE_ARGUMENTS "${MAKE_ARGUMENTS}")

if("${Vanetza_DIR}" STREQUAL "${VANETZA_BINARY_DIR}")
    set(_make_command ${MAKE_COMMAND})
    if(CMAKE_GENERATOR MATCHES "Makefiles")
        set(_make_command \$\(MAKE\))
    endif()
    add_custom_target(build_vanetza
        COMMAND ${_make_command} ${MAKE_ARGUMENTS} vanetza
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Building Vanetza (external dependency)"
        VERBATIM)
    add_dependencies(Vanetza::vanetza build_vanetza)
endif()

if("${Veins_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}/extern/veins")
    add_opp_build_target(Veins)
    add_dependencies(veins build_veins)
endif()

if("${INET_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}/extern/inet4")
    add_opp_build_target(INET)
    add_dependencies(INET build_inet)
endif()

if("${SimuLTE_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}/extern/simulte" AND WITH_SIMULTE)
    add_opp_build_target(SimuLTE)
    add_dependencies(lte build_simulte)
endif()

option(WITH_ENVMOD "Build Artery with environment model feature" ON)
option(WITH_STORYBOARD "Build Artery with storyboard feature" ON)
option(WITH_TRANSFUSION "Build Artery with transfusion feature" OFF)
option(WITH_TESTBED "Build Artery with testbed feature" OFF)

add_subdirectory(src/artery)
add_subdirectory(src/traci)
add_subdirectory(scenarios)

# sumo-launchd should be run from root directory for correct file look-up
set(SUMO_LAUNCHD_ARGUMENTS "--daemon --kill" CACHE STRING "Additional arguments passed to sumo-launchd")
string(REPLACE " " ";" _sumo_launchd_args "${SUMO_LAUNCHD_ARGUMENTS}")
add_custom_target(launch_sumo
    COMMAND ${Veins_DIR}/sumo-launchd.py ${_sumo_launchd_args}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Launching SUMO"
    VERBATIM)
