@echo off
REM Remove existing hooks
if exist .git\hooks (
    rmdir /s /q .git\hooks
)

REM Create hooks directory
mkdir .git\hooks

REM Copy hooks
xcopy /e /y hooks\* .git\hooks\

echo Git hooks installed successfully.
