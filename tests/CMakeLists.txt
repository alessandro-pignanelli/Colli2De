set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

# Use Catch2
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.8.1
)
FetchContent_MakeAvailable(catch2)

# Unit test executable
add_executable(Colli2DeTests
    test_AABB.cpp
    test_dynamicBVH.cpp
)

target_link_libraries(Colli2DeTests
    PRIVATE
    Catch2::Catch2WithMain
    colli2de
)

target_include_directories(Colli2DeTests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

add_test(NAME Colli2DeTests COMMAND Colli2DeTests)