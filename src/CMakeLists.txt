set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebugDLL" CACHE STRING "" FORCE)
    if (WIN32)
        add_compile_options(-fdiagnostics-color=always -fno-diagnostics-show-caret)
    endif()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-demangle")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" CACHE STRING "" FORCE)
endif()

# ---- Gather source files (.cpp) ----
file(GLOB_RECURSE SOURCES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# ---- Gather header files ----
file(GLOB_RECURSE HEADERS
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp
)

# ---- Add library ----
if(APPLE)
    add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
else()
    add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
endif()
set(CMAKE_DEBUG_POSTFIX "d")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

# ---- Include directories ----
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>     # during build
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>   # after install
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)
# target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)

if (MSVC)
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Colli2De symbols
		target_compile_definitions(${PROJECT_NAME} INTERFACE COLLI2DE_DLL)
	endif()

	# Warnings
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)

	if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		target_compile_options(${PROJECT_NAME} PRIVATE -Wmissing-prototypes)
	endif()
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# ---- Installation ----

# Install public and internal headers to include/colli2de
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.tpp"
)

# Install library
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install CMake config
install(
    EXPORT ${PROJECT_NAME}Config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${COLLI2DE_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
