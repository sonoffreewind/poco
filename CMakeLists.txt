cmake_minimum_required(VERSION 3.16)
project(poco)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

file(GLOB SRCS "./src/*.cpp")
find_package(Threads REQUIRED)

add_library(poco STATIC ${SRCS})

# Compile options
target_compile_options(poco PUBLIC
    $<$<CONFIG:Debug>:-g -O0 -Wall>
    $<$<CONFIG:Release>:-O3 -Wall -DNDEBUG>
)

# C++ Standard
target_compile_features(poco PUBLIC cxx_std_17)

# Include directories
target_include_directories(poco PUBLIC ./include)

# Link dependencies
target_link_libraries(poco PUBLIC Threads::Threads)

# Examples (Corresponds to Usage in README)
# Only build examples if POCO is the main project 
# (Avoids polluting the target list when used as a submodule)
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(BUILD_EXAMPLES "Build example programs" ON)

    if(BUILD_EXAMPLES)
        # Simple usage example
        add_executable(simple_examples examples/simple_examples.cpp)
        target_link_libraries(simple_examples PRIVATE poco)

        # Performance benchmark
        add_executable(benchmark examples/benchmark.cpp)
        target_link_libraries(benchmark PRIVATE poco)
    endif()
endif()

# Tests (Corresponds to Testing in README)
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(BUILD_TESTING "Build test suite" ON)

    if(BUILD_TESTING)
        enable_testing()

        # Basic functionality tests
        add_executable(test_basic tests/test_basic.cpp)
        target_link_libraries(test_basic PRIVATE poco)
        add_test(NAME TestBasic COMMAND test_basic)

        # Oracle class tests
        add_executable(test_oracle tests/test_oracle.cpp)
        target_link_libraries(test_oracle PRIVATE poco)
        add_test(NAME TestOracle COMMAND test_oracle)
    endif()
endif()