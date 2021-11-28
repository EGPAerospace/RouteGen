cmake_minimum_required (VERSION 3.16)

# Setting project name
project (
	RouteGenerator
	VERSION 1.0
	LANGUAGES CXX)

# Add C++ standard
set(CMAKE_CXX_STANDARD 17)

# Get the path where this CMake file is located
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

# Creating variables to store the path where the CMake result must be installed:
set(ROUTEGEN_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/build/include)
set(ROUTEGEN_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/build/bin)
set(ROUTEGEN_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/build/lib)

# Providing the main interface header for the library usage
set(ROUTE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/src/route)


include_directories(${ROUTEGEN_INSTALL_INCLUDE_DIR})
include_directories(${ROUTE_HEADERS_DIR})

# Adding subdirectories
add_subdirectory (ext)
add_subdirectory (src)
add_subdirectory (test)
