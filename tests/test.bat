@echo off
echo 🕷️ HEXAPOD TESTING SYSTEM (Windows)
echo =================================

REM Проверяем наличие g++
where g++ >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: g++ not found. Please install MinGW or Visual Studio.
    echo You can install MinGW from: https://www.mingw-w64.org/
    pause
    exit /b 1
)

REM Компилируем тесты
echo Compiling tests...
g++ -std=c++11 -Wall -Wextra -g -o run_tests.exe run_tests.cpp

if %errorlevel% neq 0 (
    echo ERROR: Compilation failed!
    pause
    exit /b 1
)

REM Запускаем тесты
echo.
echo Running tests...
echo.
run_tests.exe

REM Очистка
del run_tests.exe

echo.
echo Tests completed. Press any key to continue...
pause >nul
