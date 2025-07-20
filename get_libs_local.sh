#!/bin/bash

# Exit immediately on error
set -e

# Parse command-line arguments
for arg in "$@"; do
    case "$arg" in
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
            echo -e "\033[0;31m[Get Libs] Unknown option: $arg\033[0m"
            exit 1
            ;;
    esac
done

pushd "$(dirname "$0")" > /dev/null


# ------- Catch2 -------

# Git clone Catch2 if it doesn't exist
if [ ! -d "libs/catch2" ]; then
    echo -e "\033[32m[Catch2] Cloning Catch2 repository...\033[0m"
    git clone --depth 1 https://github.com/catchorg/Catch2.git libs/catch2
fi
cd libs/catch2

# Create the build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi
cd build

# Loop through the two build types
for build_type in Debug Release; do
    # Create the build type subdirectory if it doesn't exist
    if [ ! -d "$build_type" ]; then
        mkdir "$build_type"
    fi
    cd "$build_type"

    # Run cmake
    cmake ../.. -DCMAKE_BUILD_TYPE="$build_type" -DCMAKE_OSX_ARCHITECTURES=arm64
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[Catch2] CMake failed. Please check the output for details.\033[0m"
        exit 1
    fi

    # Build the project
    cmake --build . --config "$build_type"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[Catch2] Build failed. Please check the output for details.\033[0m"
        exit 1
    fi

    # Install the project
    if [ ! -d "../../install" ]; then
        mkdir -p "../../install"
    fi

    if [ -d "../../install/$build_type" ]; then
        rm -rf "../../install/$build_type"
    fi
    mkdir "../../install/$build_type"

    cmake --install . --config "$build_type" --prefix "../../install/$build_type"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[Catch2] Install failed. Please check the output for details.\033[0m"
        exit 1
    fi
    echo -e "\033[32m[Catch2] Build and installation for $build_type completed successfully.\033[0m"
    cd ..  # Go back to the build directory
done

echo -e "\033[32m[Catch2] Build and installation completed successfully.\033[0m"

# Return to the project root
popd > /dev/null

pushd "$(dirname "$0")" > /dev/null


# ------- oneTBB -------

# Git clone oneTBB if it doesn't exist
if [ ! -d "libs/oneTBB" ]; then
    echo -e "\033[32m[oneTBB] Cloning oneTBB repository...\033[0m"
    git clone --depth 1 --branch v2022.2.0 https://github.com/uxlfoundation/oneTBB.git libs/oneTBB
fi
cd libs/oneTBB

# Create the build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi
cd build

# Loop through the two build types
for build_type in Debug Release; do
    # Create the build type subdirectory if it doesn't exist
    if [ ! -d "$build_type" ]; then
        mkdir "$build_type"
    fi
    cd "$build_type"

    # Run cmake
    cmake ../.. -DCMAKE_BUILD_TYPE="$build_type" -DCMAKE_OSX_ARCHITECTURES=arm64
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[oneTBB] CMake failed. Please check the output for details.\033[0m"
        exit 1
    fi

    # Build the project
    cmake --build . --config "$build_type"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[oneTBB] Build failed. Please check the output for details.\033[0m"
        exit 1
    fi

    # Install the project
    if [ ! -d "../../install" ]; then
        mkdir -p "../../install"
    fi

    if [ -d "../../install/$build_type" ]; then
        rm -rf "../../install/$build_type"
    fi
    mkdir "../../install/$build_type"

    cmake --install . --config "$build_type" --prefix "../../install/$build_type"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[oneTBB] Install failed. Please check the output for details.\033[0m"
        exit 1
    fi
    echo -e "\033[32m[oneTBB] Build and installation for $build_type completed successfully.\033[0m"
    cd ..  # Go back to the build directory
done

echo -e "\033[32m[oneTBB] Build and installation completed successfully.\033[0m"

# Return to the project root
popd > /dev/null
