#!/bin/bash

# Exit immediately on error
set -e

pushd "$(dirname "$0")" > /dev/null

# Create the build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi
cd build

cmake_arg=0
build_type="Debug"
build_deps=0

# Parse command-line arguments
for arg in "$@"; do
    case "$arg" in
        --cmake) cmake_arg=1 ;;
        --release|-r) build_type="Release" ;;
        --debug|-d) build_type="Debug" ;;
        --profile|--relwithdebinfo|-p) build_type="RelWithDebInfo" ;;
        --build_deps|-bd) build_deps=1 ;;
        --help|-h)
            echo "Usage: $0 [--cmake] [--release|--debug|--profile|--relwithdebinfo] [--build_deps]"
            echo "Options:"
            echo "  --cmake                           Run CMake configuration"
            echo "  --release|-r                      Build in Release mode"
            echo "  --debug|-d                        Build in Debug mode"
            echo "  --profile|--relwithdebinfo|-p     Build in RelWithDebInfo mode"
            echo "  --build_deps                      Build dependencies (GameEngine)"
            exit 0
            ;;
        *)
            echo -e "\033[0;31m[ERROR] Unknown option: $arg\033[0m"
            exit 1
            ;;
    esac
done

# Create the build type subdirectory if it doesn't exist
if [ ! -d "$build_type" ]; then
    mkdir "$build_type"
fi
cd "$build_type"

# Run cmake if explicitly requested or if CMakeCache.txt doesn't exist
if [ "$cmake_arg" -eq 1 ] || [ ! -f "CMakeCache.txt" ]; then
    cmake ../.. \
        -G "Ninja" \
        -DCMAKE_CXX_COMPILER=g++ \
        -DCMAKE_INSTALL_PREFIX="../../install/$build_type" \
        -DCMAKE_BUILD_TYPE=$build_type \
        -DCMAKE_OSX_ARCHITECTURES=arm64
fi

# Build and install
if [ -d "../../install/include" ]; then
    rm -r ../../install/include
fi
cmake --build . --target install --config $build_type

# Return to the project root
popd > /dev/null
