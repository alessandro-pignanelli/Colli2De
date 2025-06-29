@echo off

pushd "%~dp0"
SETLOCAL ENABLEDELAYEDEXPANSION
REM ---------- Parse command line arguments ----------

SET build=0
SET cmake_arg=0
SET build_type=Debug
SET lower_build_type=debug
SET run_until_failure=0
SET cmd_args=

FOR %%A IN (%*) DO (
    SET allowed=0
    IF "%%A"=="--build" (
        SET build=1
        SET allowed=1
    )
    IF "%%A"=="-b" (
        SET build=1
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
    IF "%%A"=="--run-until-failure" (
        SET run_until_failure=1
        SET allowed=1
    )
    IF "%%A"=="-ruf" (
        SET run_until_failure=1
        SET allowed=1
    )
    IF "%%A"=="--help" GOTO PRINT_HELP
    IF "%%A"=="-h" GOTO PRINT_HELP

    IF !allowed! EQU 0 (
        SET cmd_args=!cmd_args! %%A
    )
)

REM ---------- Build ----------

IF %build%==1 (
    set args=
    if "%build_type%"=="Release" (
        set args=--release
    ) else if "%build_type%"=="Debug" (
        set args=--debug
    ) else if "%build_type%"=="RelWithDebInfo" (
        set args=--profile
    )

    if %cmake_arg%==1 (
        set args=!args! --cmake
    )

    echo Building project with arguments: !args! ^(build type: %build_type%^)
    pushd "%~dp0" >nul

    call build.bat !args!
    IF !ERRORLEVEL! NEQ 0 (
        exit /b %ERRORLEVEL%
    )

    popd >nul
    echo Build completed successfully.
)

REM ---------- Run Tests ----------
cd build\%build_type%\tests

:RUN_TESTS

call Colli2DeTests.exe %cmd_args%
IF %ERRORLEVEL% NEQ 0 (
    echo [31m[Colli2De] Tests failed![0m
    exit /b %ERRORLEVEL%
)

IF %run_until_failure%==1 (
    echo [32m[Colli2De] All tests passed successfully.[0m
    echo.
    echo [33m[Colli2De] Running again...[0m
    echo.
    cls
    GOTO RUN_TESTS
)

echo [32m[Colli2De] All tests passed successfully.[0m
echo.
