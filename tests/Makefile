# Makefile для системы тестирования гексапода

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -g
TARGET = run_tests
SOURCES = run_tests.cpp

# Основная цель
all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES)

# Запуск тестов
test: $(TARGET)
	./$(TARGET)

# Запуск тестов в режиме отладки
debug: $(TARGET)
	gdb ./$(TARGET)

# Очистка
clean:
	rm -f $(TARGET)

# Быстрый тест (компиляция + запуск)
quick: clean all test

.PHONY: all test debug clean quick
