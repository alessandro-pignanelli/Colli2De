@echo off

pushd "%~dp0"
SETLOCAL ENABLEDELAYEDEXPANSION

FOR %%A IN (%*) DO (
    IF "%%A"=="--help" GOTO PRINT_HELP
    IF "%%A"=="-h" GOTO PRINT_HELP
)



REM ---------- Main execution ----------
call :FETCH_AND_BUILD_LIB "Catch2" "catch2" "https://github.com/catchorg/Catch2.git"
IF %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

call :FETCH_AND_BUILD_LIB "Robin Map" "robin-map" "https://github.com/Tessil/robin-map.git"
IF %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

popd >nul
exit /b 0


REM ---------- FETCH_AND_BUILD_LIB ----------
REM Parameters:
REM   %1 - Display name (e.g., "Catch2")
REM   %2 - Directory name (e.g., "catch2")
REM   %3 - Git repository URL
REM   %4 - Skip build (1 to skip, 0 to build)
:FETCH_AND_BUILD_LIB
SET LIB_NAME=%~1
SET LIB_DIR=%~2
SET GIT_URL=%~3
SET SKIP_BUILD=%~4

echo [34m==================== Fetching %LIB_NAME% ====================[0m

REM ---------- Git clone ----------
IF NOT EXIST "libs/%LIB_DIR%" (
    echo [32m[%LIB_NAME%] Cloning repository...[0m
    git clone %GIT_URL% libs/%LIB_DIR%
)

REM ---------- Check if build should be skipped ----------
IF "%SKIP_BUILD%"=="1" (
    echo [33m[%LIB_NAME%] Skipping build[0m
    echo.
    exit /b 0
)

cd libs/%LIB_DIR%

if not exist "build" (
    mkdir "build"
)
cd build

REM ---------- Build for Debug ----------
call :BUILD_AND_INSTALL "%LIB_NAME%" "Debug"
IF %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

REM ---------- Build for Release ----------
call :BUILD_AND_INSTALL "%LIB_NAME%" "Release"
IF %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cd ..\..\..
exit /b 0


REM ---------- BUILD_AND_INSTALL ----------
REM Parameters:
REM   %1 - Library name
REM   %2 - Build type (Debug/Release)
:BUILD_AND_INSTALL
SET LIB_NAME=%~1
SET BUILD_TYPE=%~2

IF NOT EXIST "%BUILD_TYPE%" (
    mkdir "%BUILD_TYPE%"
)
cd "%BUILD_TYPE%"

REM ---------- Run CMake ----------
echo [32m[%LIB_NAME%] Configuring with CMake (%BUILD_TYPE%)...[0m
cmake ../.. ^
    -G "Ninja" ^
    -DCMAKE_C_COMPILER=gcc ^
    -DCMAKE_CXX_COMPILER=g++ ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[%LIB_NAME%] CMake failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

REM ---------- Build the project ----------
echo [32m[%LIB_NAME%] Building project with build type: %BUILD_TYPE%[0m
cmake --build . --config %BUILD_TYPE%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[%LIB_NAME%] Build failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

REM ---------- Install ----------
IF NOT EXIST "../../install" (
    mkdir "../../install"
)

IF EXIST "../../install/%BUILD_TYPE%" (
    rmdir /s /q "../../install/%BUILD_TYPE%"
)
mkdir "../../install/%BUILD_TYPE%"

echo [32m[%LIB_NAME%] Installing...[0m
cmake --install . --config %BUILD_TYPE% --prefix "../../install/%BUILD_TYPE%"
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[%LIB_NAME%] Installation failed. Please check the output for details.[0m
    exit /b %ERRORLEVEL%
)

echo [32m[%LIB_NAME%] CMake completed successfully (%BUILD_TYPE%).[0m
echo.

cd ..
exit /b 0


REM ---------- PRINT_HELP ----------
:PRINT_HELP

echo Usage: get_libs_local.bat [--cmake] [--release^|--debug^|--profile^|--relwithdebinfo] [--build_deps]
echo Options:
echo     --cmake                            Run CMake configuration
echo     --release^|-r                      Build in Release mode
echo     --debug^|-d                        Build in Debug mode
echo     --profile^|--relwithdebinfo|-p     Build in RelWithDebInfo mode
echo     --build_deps                       Build dependencies (GameEngine)
echo     --help^|-h                         Show this help message
exit /b 0

REM ---------- PRINT_HELP ----------
