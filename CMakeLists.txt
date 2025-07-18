set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.22)

project(colli2de
	VERSION 1.0.0
	DESCRIPTION "2D Collision Detection system"
	LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# static link
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Deterministic math
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

set (UNIT_TESTS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

message(STATUS "Building Colli2De version ${PROJECT_VERSION} with C++ standard ${CMAKE_CXX_STANDARD}")
add_subdirectory(src)

if(UNIT_TESTS AND (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR))
	message(STATUS "Building unit tests")
	add_subdirectory(tests)
else()
	message(STATUS "Skipping unit tests")
endif()

message(STATUS "Colli2De build complete")
