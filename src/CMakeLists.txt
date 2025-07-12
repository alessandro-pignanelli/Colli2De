set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Gather source files (.cpp) ----
file(GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/colli2de/internal/**/*.cpp
)

# ---- Gather internal and public header files ----
file(GLOB_RECURSE INTERNAL_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/colli2de/internal/**/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/colli2de/internal/**/*.tpp
)

file(GLOB_RECURSE PUBLIC_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/colli2de/public/*.hpp
)

# ---- Add library ----
add_library(colli2de ${SOURCES} ${INTERNAL_HEADERS} ${PUBLIC_HEADERS})

# ---- Include directories ----
target_include_directories(colli2de
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/colli2de/internal
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

# ---- Create alias ----
add_library(colli2de::colli2de ALIAS colli2de)

# ---- Installation ----

# Install public and internal headers to include/colli2de
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/colli2de
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.tpp"
)

# Install library
install(
    TARGETS colli2de
    EXPORT colli2deConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install CMake config
install(
    EXPORT colli2deConfig
    NAMESPACE colli2de::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/colli2de"
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
