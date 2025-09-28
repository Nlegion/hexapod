#!/bin/bash

echo "🕷️ HEXAPOD TESTING SYSTEM (Linux/Mac)"
echo "===================================="

# Проверяем наличие g++
if ! command -v g++ &> /dev/null; then
    echo "ERROR: g++ not found. Please install it:"
    echo "Ubuntu/Debian: sudo apt install g++"
    echo "CentOS/RHEL:   sudo yum install gcc-c++"
    echo "MacOS:         brew install gcc"
    exit 1
fi

# Компилируем тесты
echo "Compiling tests..."
g++ -std=c++11 -Wall -Wextra -g -o run_tests run_tests.cpp

if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed!"
    exit 1
fi

# Запускаем тесты
echo ""
echo "Running tests..."
echo ""
./run_tests

# Очистка
rm -f run_tests

echo ""
echo "Tests completed."
