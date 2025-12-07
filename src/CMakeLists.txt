set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DBG)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebugDLL" CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-demangle")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" CACHE STRING "" FORCE)
endif()

# Include custom CMake modules
include(IncludePackageWithFallback)

# Include required packages
include_package_with_fallback(
    tsl-robin-map
    robin-map
    "https://github.com/Tessil/robin-map.git"
    1.4.0
    USE_LOCAL_ROBIN_MAP
)
if (NOT USE_LOCAL_ROBIN_MAP)
    set(ROBIN_MAP_TARGET robin_map)
endif()

if (C2D_USE_CEREAL)
    message(STATUS "Cereal serialization support enabled")

    include_package_with_fallback(
        cereal
        cereal
        "https://github.com/USCiLab/cereal.git"
        1.3.2
        USE_LOCAL_CEREAL
        JUST_INSTALL_CEREAL ON
    )
    if (NOT USE_LOCAL_CEREAL)
        set(CEREAL_TARGET cereal)
    endif()
endif()

# ---- Gather source files (.cpp) ----
file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# ---- Gather header files ----
file(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp)

# ---- Add library ----
if(APPLE)
    add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
else()
    add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
endif()
set(CMAKE_DEBUG_POSTFIX "d")

if (C2D_USE_CEREAL)
    target_compile_definitions(${PROJECT_NAME} PUBLIC C2D_USE_CEREAL)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
        cereal::cereal
    )
endif()

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    tsl::robin_map
)

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

if(C2D_ENABLE_LTO)
    message(STATUS "IPO / LTO enabled")
    set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Add support for std::execution::par
if (MSVC)
    # Nothing needed, works out of the box
elseif (APPLE)
    # Disabled
elseif (UNIX)
    # GCC: enable OpenMP backend
    target_compile_options(${PROJECT_NAME} PRIVATE -fopenmp)
    target_link_libraries(${PROJECT_NAME} PRIVATE gomp) # or omp
endif()

# ---- Include directories ----
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>     # during build
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>         # after install
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)
target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)

if (MSVC)
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Colli2De symbols
		target_compile_definitions(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_DLL)
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

# Install public headers to include/colli2de
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "internal" EXCLUDE
)

# Install robin-map headers (dependency of colli2de)
if (USE_LOCAL_ROBIN_MAP)
    install(
        DIRECTORY ${COLLI2DE_SOURCE_DIR}/libs/robin-map/include/
        DESTINATION include
    )
else()
    # When using FetchContent, install from the fetched source
    install(
        DIRECTORY ${tsl-robin-map_SOURCE_DIR}/include/
        DESTINATION include
    )
endif()

# Install library
install(
    TARGETS ${PROJECT_NAME} ${ROBIN_MAP_TARGET} ${CEREAL_TARGET}
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
