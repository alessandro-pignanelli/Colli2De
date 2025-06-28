@echo off

pushd "%~dp0"
SETLOCAL ENABLEDELAYEDEXPANSION


REM ---------- Git clone ----------
IF NOT EXIST "libs/catch2" (
    echo [32m[Catch2] Cloning Catch2 repository...[0m
    git clone https://github.com/catchorg/Catch2.git libs/catch2
)

cd libs/catch2

if not exist "build" (
    mkdir "build"
)
cd build


REM ---------- Parse command line arguments ----------

SET build_type=Debug
SET lower_build_type=debug

FOR %%A IN (%*) DO (
    SET allowed=0
    IF "%%A"=="--release" (
        SET build_type=Release
        SET lower_build_type=release
        SET allowed=1
    )
    IF "%%A"=="-r" (
        SET build_type=Release
        SET lower_build_type=release
        SET allowed=1
    )
    IF "%%A"=="--debug" (
        SET build_type=Debug
        SET lower_build_type=debug
        SET allowed=1
    )
    IF "%%A"=="-d" (
        SET build_type=Debug
        SET lower_build_type=debug
        SET allowed=1
    )
    IF "%%A"=="--profile" (
        SET build_type=RelWithDebInfo
        SET lower_build_type=relwithdebinfo
        SET allowed=1
    )
    IF "%%A"=="--relwithdebinfo" (
        SET build_type=RelWithDebInfo
        SET lower_build_type=relwithdebinfo
        SET allowed=1
    )
    IF "%%A"=="-p" (
        SET build_type=RelWithDebInfo
        SET lower_build_type=relwithdebinfo
        SET allowed=1
    )
    IF "%%A"=="--help" GOTO PRINT_HELP
    IF "%%A"=="-h" GOTO PRINT_HELP

    IF !allowed! EQU 0 (
        echo [31m[Catch2] Unknown option: %%A[0m
        GOTO PRINT_HELP
    )
)


REM ---------- Run CMake ----------

IF NOT EXIST "%build_type%" (
    mkdir "%build_type%"
)
cd "%build_type%"

cmake ../.. ^
    -G "Ninja" ^
    -DCMAKE_CXX_COMPILER=g++ ^
    -DCMAKE_BUILD_TYPE=%build_type%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Catch2] CMake failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

REM ---------- Build the project ----------

echo [32m[Catch2] Building project with build type: %build_type%[0m
cmake --build . --config %build_type% --parallel 8
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Catch2] Build failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

REM ---------- Install Catch2 ----------

IF NOT EXIST "../../install" (
    mkdir "../../install"
)

IF EXIST "../../install/%build_type%" (
    rmdir /s /q "../../install/%build_type%"
)
mkdir "../../install/%build_type%"

echo [32m[Catch2] Installing Catch2...[0m
cmake --install . --config %build_type% --prefix "../../install/%build_type%"
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Catch2] Installation failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

echo [32m[Catch2] CMake completed successfully.[0m
echo.

popd >nul
exit /b 0


REM ---------- PRINT_HELP ----------
:PRINT_HELP

echo Usage: get_catch2_local.bat [--cmake] [--release^|--debug^|--profile^|--relwithdebinfo] [--build_deps]
echo Options:
echo     --cmake                            Run CMake configuration
echo     --release^|-r                      Build in Release mode
echo     --debug^|-d                        Build in Debug mode
echo     --profile^|--relwithdebinfo|-p     Build in RelWithDebInfo mode
echo     --build_deps                       Build dependencies (GameEngine)
echo     --help^|-h                         Show this help message
exit /b 0

REM ---------- PRINT_HELP ----------
