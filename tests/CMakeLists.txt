set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

include(CTest)
enable_testing()

# Include custom CMake modules
include(IncludePackageWithFallback)

# Use Catch2
include_package_with_fallback(
    Catch2
    catch2
    "https://github.com/catchorg/Catch2.git"
    3.11.0
    USE_LOCAL_CATCH2
)

file(GLOB_RECURSE TESTS "test_*.cpp")

# Unit test executable
add_executable(Colli2DeTests ${TESTS} ${CMAKE_SOURCE_DIR}/tests/main.cpp)

set_target_properties(Colli2DeTests PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(Colli2DeTests
    PRIVATE
    Catch2::Catch2
    colli2de
)

if (WIN32)
    target_link_libraries(Colli2DeTests
        PRIVATE
        stdc++exp # std::print
    )
endif()

if(C2D_ENABLE_LTO)
    message(STATUS "IPO / LTO enabled")
    set_property(TARGET Colli2DeTests PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

target_include_directories(Colli2DeTests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/tests
)

add_test(NAME Colli2DeTests COMMAND Colli2DeTests)
