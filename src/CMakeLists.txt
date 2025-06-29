set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/*.hpp
)

file(GLOB_RECURSE API_FILES
    ${CMAKE_SOURCE_DIR}/include/colli2de/*.hpp
)

add_library(colli2de ${SOURCES} ${API_FILES})

target_include_directories(colli2de
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_DEBUG_POSTFIX "d")

if (MSVC)
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Colli2De symbols
		target_compile_definitions(colli2de INTERFACE COLLI2DE_DLL)
	endif()

	# Warnings
	# 4710 - warn about inline functions that are not inlined
	target_compile_options(colli2de PRIVATE /Wall /wd4820 /wd5045 /wd4061 /wd4711 /wd4710)

	if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		target_compile_options(colli2de PRIVATE -Wmissing-prototypes)
	endif()
elseif (MINGW)
elseif (APPLE)
	target_compile_options(colli2de PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Werror)
elseif (EMSCRIPTEN)
	target_compile_options(colli2de PRIVATE -msimd128 -msse2)
elseif (UNIX)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COLLI2DE_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${COLLI2DE_API_FILES})

add_library(colli2de::colli2de ALIAS colli2de)

install(
  TARGETS colli2de
  EXPORT colli2deConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT colli2deConfig
  NAMESPACE colli2de::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/colli2de"
)

install(
  FILES ${COLLI2DE_API_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/colli2de
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/colli2deConfigVersion.cmake"
  VERSION ${COLLI2DE_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/colli2deConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/colli2de"
)
