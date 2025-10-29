@echo off
setlocal enabledelayedexpansion

REM compare_tests.bat
REM Merge two or more benchmark markdown tables (same format, arbitrary row order)
REM into a single markdown table. Benchmarks are sorted by name and, for each
REM input file N, columns are prefixed with "N." (e.g., 1.Status, 2.Avg (ms), ...).
REM
REM Usage:
REM   compare_tests.bat [-o output.md] [-n COUNT] [file1.md file2.md ...]
REM If -n is provided, the script ignores positional files and merges the last COUNT
REM benchmark files from test_data (newest first). If -o is not provided, the merged
REM table is printed to stdout.

set "output_file="
set "ncount="
set "file_list="
set "file_count=0"

REM Parse arguments
:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="-o" (
    set "output_file=%~2"
    shift
    shift
    goto parse_args
)
if /i "%~1"=="-n" (
    set "ncount=%~2"
    shift
    shift
    goto parse_args
)
if /i "%~1"=="-h" (
    call :usage
    exit /b 0
)
if /i "%~1"=="/?" (
    call :usage
    exit /b 0
)
REM It's a file argument
if not "!file_list!"=="" (
    set "file_list=!file_list! %~1"
) else (
    set "file_list=%~1"
)
set /a file_count+=1
shift
goto parse_args
:end_parse

REM Process -n option
if not "!ncount!"=="" (
    REM Validate ncount is a positive integer >= 2
    set /a test_n=!ncount! 2>nul
    if !test_n! LSS 2 (
        echo Error: -n COUNT must be at least 2 1>&2
        exit /b 1
    )
    
    REM Collect last COUNT benchmark files
    set "temp_list=%TEMP%\benchmark_list_%RANDOM%.txt"
    dir /b /o:n ..\test_data\benchmarks--*.md 2>nul > "!temp_list!"
    
    REM Count available files
    set /a available=0
    set /a line_num=0
    for /f %%F in ('type "!temp_list!" ^| find /c /v ""') do set /a available=%%F
    
    if !available! LSS !ncount! (
        echo Error: found fewer than !ncount! benchmark files matching pattern in test_data 1>&2
        del "!temp_list!" 2>nul
        exit /b 1
    )
    
    REM Get last n files (newest first by reversing)
    set "file_list="
    set /a skip_count=!available!-!ncount!
    set /a idx=0
    for /f "usebackq delims=" %%F in ("!temp_list!") do (
        set /a line_num+=1
        if !line_num! GTR !skip_count! (
            set "files_arr[!idx!]=..\test_data\%%F"
            set /a idx+=1
        )
    )
    
    REM Reverse the array
    set /a last=!idx!-1
    set "file_list="
    for /l %%i in (!last!,-1,0) do (
        if "!file_list!"=="" (
            set "file_list=!files_arr[%%i]!"
        ) else (
            set "file_list=!file_list! !files_arr[%%i]!"
        )
    )
    set /a file_count=!ncount!
    del "!temp_list!" 2>nul
) else (
    if !file_count! LSS 2 (
        echo Error: need at least two input files or use -n COUNT 1>&2
        call :usage
        exit /b 1
    )
    
    REM Prepend test_data/ to provided paths unless already present
    set "new_file_list="
    for %%F in (!file_list!) do (
        set "filepath=%%F"
        echo !filepath! | findstr /b /c:"test_data\" >nul
        if errorlevel 1 (
            set "filepath=test_data\!filepath!"
        )
        if "!new_file_list!"=="" (
            set "new_file_list=!filepath!"
        ) else (
            set "new_file_list=!new_file_list! !filepath!"
        )
    )
    set "file_list=!new_file_list!"
)

REM Validate all input files exist
for %%F in (!file_list!) do (
    if not exist "%%F" (
        echo Error: file not found: %%F 1>&2
        exit /b 1
    )
)

REM Create temporary files
set "tmp_data=%TEMP%\compare_tests_data_%RANDOM%.txt"
set "tmp_names=%TEMP%\compare_tests_names_%RANDOM%.txt"

REM Parse files using Python
REM First, check if Python is available
where python >nul 2>&1
if errorlevel 1 (
    echo Error: Python is required but not found in PATH 1>&2
    exit /b 1
)

REM Check if parse script exists
if not exist "compare_tests_parse.py" (
    echo Error: compare_tests_parse.py not found 1>&2
    exit /b 1
)

REM Run Python script to parse files
REM Build command with properly quoted file paths
set "parse_cmd=python compare_tests_parse.py "!tmp_data!""
for %%F in (!file_list!) do (
    set "parse_cmd=!parse_cmd! "%%F""
)
!parse_cmd!

REM Extract and sort unique benchmark names using Python directly
python -c "import sys; names = set(); [names.add(line.split('\t')[2]) for line in open(sys.argv[1], 'r', encoding='utf-8') if len(line.split('\t')) > 2]; [print(n) for n in sorted(names)]" "!tmp_data!" > "!tmp_names!"

REM Check if format script exists
if not exist "compare_tests_format.py" (
    echo Error: compare_tests_format.py not found 1>&2
    exit /b 1
)

REM Generate output
if not "!output_file!"=="" (
    REM Ensure output file is in test_data if not already
    echo !output_file! | findstr /b /c:"test_data\" >nul
    if errorlevel 1 (
        set "output_file=..\test_data\!output_file!"
    )
    python compare_tests_format.py "!tmp_data!" "!tmp_names!" !file_count! > "!output_file!"
) else (
    python compare_tests_format.py "!tmp_data!" "!tmp_names!" !file_count!
)

REM Cleanup
if exist "!tmp_data!" del "!tmp_data!" 2>nul
if exist "!tmp_names!" del "!tmp_names!" 2>nul

exit /b 0

:usage
echo Usage: %~nx0 [-o output.md] [-n COUNT] [file1.md file2.md ...] 1>&2
echo   -n COUNT  Merge the last COUNT files from test_data matching benchmarks--*.md ^(newest first^). 1>&2
exit /b 0
