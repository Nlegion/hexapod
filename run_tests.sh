#!/bin/bash

echo "🕷️ HEXAPOD TESTING SYSTEM (Linux/Mac)"
echo "===================================="
echo "Running from: $(pwd)"

# Проверяем наличие папки tests
if [ ! -d "tests" ]; then
    echo "ERROR: Tests directory not found!"
    echo "Make sure you are in the hexapod project root directory."
    exit 1
fi

# Переходим в папку tests и запускаем тестирование
cd tests
./test.sh

# Возвращаемся в корневую папку
cd ..

echo ""
echo "Back in project root: $(pwd)"
