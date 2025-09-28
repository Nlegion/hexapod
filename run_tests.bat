@echo off
echo 🕷️ HEXAPOD TESTING SYSTEM (Windows)
echo =================================
echo Running from: %CD%

REM Проверяем наличие папки tests
if not exist "tests" (
    echo ERROR: Tests directory not found!
    echo Make sure you are in the hexapod project root directory.
    pause
    exit /b 1
)

REM Переходим в папку tests и запускаем тестирование
cd tests
call test.bat

REM Возвращаемся в корневую папку
cd ..

echo.
echo Back in project root: %CD%
