#!/bin/bash

# Exit immediately on error
set -e
pushd "$(dirname "$0")" > /dev/null

cmake_arg=0
build_type="Debug"
build=0
args=()

# Parse command-line arguments
for arg in "$@"; do
    case "$arg" in
        --cmake) cmake_arg=1 ;;
        --release|-r) build_type="Release" ;;
        --debug|-d) build_type="Debug" ;;
        --profile|--relwithdebinfo|-p) build_type="RelWithDebInfo" ;;
        --build|-b) build=1 ;;
        --help|-h)
            echo "Usage: $0 [--cmake] [--release|--debug|--profile|--relwithdebinfo] [--build]"
            echo "Options:"
            echo "  --cmake                           Run CMake configuration"
            echo "  --release|-r                      Build in Release mode"
            echo "  --debug|-d                        Build in Debug mode"
            echo "  --profile|--relwithdebinfo|-p     Build in RelWithDebInfo mode"
            echo "  --build                           Build (Colli2De)"
            exit 0
            ;;
        *)
            args+=("$arg")
            ;;
    esac
done

# Create the build type subdirectory if it doesn't exist
if [[ $build -eq 1 ]]; then
    build_args=()
    if [[ $build_type == "Release" ]]; then
        build_args+=("--release")
    elif [[ $build_type == "Debug" ]]; then
        build_args+=("--debug")
    elif [[ $build_type == "RelWithDebInfo" ]]; then
        build_args+=("--profile")
    fi
    if [[ $cmake_arg -eq 1 ]]; then
        build_args+=("--cmake")
    fi
    if [[ $build_deps -eq 1 ]]; then
        build_args+=("--build_deps")
    fi

    echo "[Colli2De] Building project with arguments: ${build_args[*]}"
    pushd "$(dirname "$0")" > /dev/null
    ./build.sh "${build_args[@]}"
    popd > /dev/null
    echo -e "\033[0;32m[Colli2De] Build completed successfully.\033[0m"
fi

if [[ ! -d "build" ]]; then
    echo "Project build directory does not exist."
    exit 1
fi
if [[ ! -d "build/$build_type" ]]; then
    echo "Build type directory '$build_type' does not exist in 'build'."
    exit 1
fi
if [[ ! -d "build/$build_type/tests" ]]; then
    echo "Build type directory 'tests' does not exist in 'build/$build_type'."
    exit 1
fi

# Run the tests executable
executable_path="build/$build_type/tests/Colli2DeTests"
if [[ -f "$executable_path" ]]; then
    current_dir=$(pwd)
    "$executable_path" "${args[@]}" "$current_dir/test_data/"
    echo
    if [[ $? -eq 0 ]]; then
        echo -e "\033[0;32m[Colli2De] All tests passed successfully.\033[0m"
    else
        echo -e "\033[0;31m[Colli2De] Some tests failed.\033[0m"
        exit 1
    fi
else
    echo -e "\033[0;31m[Colli2De] Test executable not found at '$executable_path'.\033[0m"
    exit 1
fi

# Return to the project root
popd > /dev/null
