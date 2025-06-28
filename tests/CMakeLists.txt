set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

# Use Catch2
if (USE_LOCAL_CATCH2)
    find_package(Catch2 3.8 REQUIRED PATHS ${CMAKE_SOURCE_DIR}/libs/catch2/install/${CMAKE_BUILD_TYPE})
else()
    find_package(Catch2 3.8 QUIET PATHS ${CMAKE_SOURCE_DIR}/libs/catch2/install/${CMAKE_BUILD_TYPE})
    if (NOT Catch2_FOUND)
        message(STATUS "Catch2 not found, fetching from GitHub")
        include(FetchContent)
        FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        v3.8.1
        )
        FetchContent_MakeAvailable(Catch2)
    else()
        message(STATUS "Using local Catch2 from ${Catch2_DIR}")
    endif()
endif()

file(GLOB_RECURSE TESTS "test_*.cpp")

# Unit test executable
add_executable(Colli2DeTests ${TESTS})

target_link_libraries(Colli2DeTests
    PRIVATE
    Catch2::Catch2WithMain
    colli2de
    stdc++exp # std::print
)

target_include_directories(Colli2DeTests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/tests
)

add_test(NAME Colli2DeTests COMMAND Colli2DeTests)