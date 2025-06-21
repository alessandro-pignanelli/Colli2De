@echo off

pushd "%~dp0"
SETLOCAL ENABLEDELAYEDEXPANSION

if not exist "build" (
    mkdir "build"
)
cd build


REM ---------- Parse command line arguments ----------

SET cmake_arg=0
SET build_type=Debug
SET lower_build_type=debug
SET build_deps=0

FOR %%A IN (%*) DO (
    SET allowed=0
    IF "%%A"=="--build_deps" (
        SET build_deps=1
        SET allowed=1
    )
    IF "%%A"=="-bd" (
        SET build_deps=1
        SET allowed=1
    )
    IF "%%A"=="--cmake" (
        SET cmake_arg=1
        SET allowed=1
    )
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
        echo [31m[Game] Unknown option: %%A[0m
        GOTO PRINT_HELP
    )
)


REM ---------- Build Colli2De ----------

IF NOT EXIST "%build_type%" (
    mkdir "%build_type%"
)
cd "%build_type%"

IF EXIST "CMakeCache.txt" IF %cmake_arg%==0 (
    GOTO BUILD
)

:CMAKE
cmake ../.. ^
    -G "Ninja" ^
    -DCMAKE_CXX_COMPILER=g++ ^
    -DCMAKE_BUILD_TYPE=%build_type%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Colli2De] CMake failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

:BUILD
cmake --build . --config %build_type%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Colli2De] Build failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

echo [32m[Colli2De] Build completed successfully.[0m
echo.

popd >nul
exit /b 0


REM ---------- PRINT_HELP ----------
:PRINT_HELP

echo Usage: build.bat [--cmake] [--release^|--debug^|--profile^|--relwithdebinfo] [--build_deps]
echo Options:
echo     --cmake                            Run CMake configuration
echo     --release^|-r                      Build in Release mode
echo     --debug^|-d                        Build in Debug mode
echo     --profile^|--relwithdebinfo|-p     Build in RelWithDebInfo mode
echo     --build_deps                       Build dependencies (GameEngine)
echo     --help^|-h                         Show this help message
exit /b 0

REM ---------- PRINT_HELP ----------
