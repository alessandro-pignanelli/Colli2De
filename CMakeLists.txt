set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.22)

project(Colli2De
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

add_subdirectory(src)

if (UNIT_TESTS)
	add_subdirectory(tests)
endif()
