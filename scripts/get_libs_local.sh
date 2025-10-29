#!/bin/bash

# Exit immediately on error
set -e

# Parse command-line arguments
for arg in "$@"; do
    case "$arg" in
        --help|-h)
            echo "Usage: $0 [--help|-h]"
            echo "Options:"
            echo "  --help|-h     Show this help message"
            exit 0
            ;;
        *)
            echo -e "\033[0;31m[Get Libs] Unknown option: $arg\033[0m"
            exit 1
            ;;
    esac
done

pushd "$(dirname "$0")" > /dev/null

# ---------- FETCH_AND_BUILD_LIB ----------
# Parameters:
#   $1 - Display name (e.g., "Catch2")
#   $2 - Directory name (e.g., "catch2")
#   $3 - Git repository URL
#   $4 - Skip build (1 to skip, empty/0 to build)
fetch_and_build_lib() {
    local LIB_NAME="$1"
    local LIB_DIR="$2"
    local GIT_URL="$3"
    local SKIP_BUILD="$4"

    echo -e "\033[34m==================== Fetching $LIB_NAME ====================\033[0m"

    # ---------- Git clone ----------
    if [ ! -d "libs/$LIB_DIR" ]; then
        echo -e "\033[32m[$LIB_NAME] Cloning repository...\033[0m"
        git clone "$GIT_URL" "libs/$LIB_DIR"
    fi

    # ---------- Check if build should be skipped ----------
    if [ "$SKIP_BUILD" = "1" ]; then
        echo -e "\033[33m[$LIB_NAME] Skipping build\033[0m"
        echo
        return 0
    fi

    cd "libs/$LIB_DIR"

    if [ ! -d "build" ]; then
        mkdir build
    fi
    cd build

    # ---------- Build for Debug ----------
    build_and_install "$LIB_NAME" "Debug"
    if [ $? -ne 0 ]; then
        return 1
    fi

    # ---------- Build for Release ----------
    build_and_install "$LIB_NAME" "Release"
    if [ $? -ne 0 ]; then
        return 1
    fi

    cd ../../..
    return 0
}

# ---------- BUILD_AND_INSTALL ----------
# Parameters:
#   $1 - Library name
#   $2 - Build type (Debug/Release)
build_and_install() {
    local LIB_NAME="$1"
    local BUILD_TYPE="$2"

    if [ ! -d "$BUILD_TYPE" ]; then
        mkdir "$BUILD_TYPE"
    fi
    cd "$BUILD_TYPE"

    # ---------- Run CMake ----------
    echo -e "\033[32m[$LIB_NAME] Configuring with CMake ($BUILD_TYPE)...\033[0m"
    cmake ../.. \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_OSX_ARCHITECTURES=arm64
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[$LIB_NAME] CMake failed. Please check the output for details.\033[0m"
        return 1
    fi

    # ---------- Build the project ----------
    echo -e "\033[32m[$LIB_NAME] Building project with build type: $BUILD_TYPE\033[0m"
    cmake --build . --config "$BUILD_TYPE"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[$LIB_NAME] Build failed. Please check the output for details.\033[0m"
        return 1
    fi

    # ---------- Install ----------
    if [ ! -d "../../install" ]; then
        mkdir -p "../../install"
    fi

    if [ -d "../../install/$BUILD_TYPE" ]; then
        rm -rf "../../install/$BUILD_TYPE"
    fi
    mkdir "../../install/$BUILD_TYPE"

    echo -e "\033[32m[$LIB_NAME] Installing...\033[0m"
    cmake --install . --config "$BUILD_TYPE" --prefix "../../install/$BUILD_TYPE"
    if [ $? -ne 0 ]; then
        echo -e "\033[31m[$LIB_NAME] Installation failed. Please check the output for details.\033[0m"
        return 1
    fi

    echo -e "\033[32m[$LIB_NAME] CMake completed successfully ($BUILD_TYPE).\033[0m"
    echo

    cd ..
    return 0
}

# ---------- Main execution ----------
fetch_and_build_lib "Catch2" "catch2" "https://github.com/catchorg/Catch2.git"
if [ $? -ne 0 ]; then exit 1; fi

fetch_and_build_lib "Robin Map" "robin-map" "https://github.com/Tessil/robin-map.git"
if [ $? -ne 0 ]; then exit 1; fi

popd > /dev/null
exit 0
