cmake_minimum_required(VERSION 3.5)

project(kinova_driver VERSION 2.6.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)
option(DOWNLOAD_API "Automatically download the API if conan is not used" ON)

# Activate C++ 11
set (CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

add_compile_options(-Wall)
add_compile_options(-Wno-reorder)

add_definitions(-D_OS_UNIX)

include(${PROJECT_SOURCE_DIR}/thirdParty/conan.cmake)

conan_check(REQUIRED)
conan_add_remote(
  NAME kinova_public 
  URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-local-public)

conan_cmake_run(REQUIRES kortex_api_cpp/2.7.0-r.5@kortex/stable
                SETTINGS kortex_api_cpp:compiler=gcc
                SETTINGS kortex_api_cpp:compiler.version=5
                SETTINGS compiler.libcxx=libstdc++11
                PROFILE_AUTO build_type
                BASIC_SETUP
                UPDATE)


link_libraries(${CONAN_LIBS})
link_libraries(pthread)

find_package(ruckig CONFIG REQUIRED)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(pinocchio REQUIRED)

# nlohmann/json
find_package(nlohmann_json 3.2.0 REQUIRED)

set(KINOVA_DRIVER_BIN_NAME "kinova_driver")
add_executable(${KINOVA_DRIVER_BIN_NAME} src/kinova_driver.cpp src/utilities.cpp include/redis_keys.hpp)

set(COMMON_LIBRARIES
	${EIGEN3_LIBRARIES}
    ${HIREDIS_LIBRARY}
    pinocchio::pinocchio
    ruckig
    nlohmann_json::nlohmann_json
	)

target_link_libraries(${KINOVA_DRIVER_BIN_NAME} PRIVATE ${COMMON_LIBRARIES})
