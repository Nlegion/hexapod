#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <cassert>

// Базовый фреймворк для юнит-тестов
class TestFramework {
private:
    static std::vector<std::function<void()>> tests;
    static std::string current_test_name;
    static int passed_tests;
    static int failed_tests;
    
public:
    static void add_test(const std::string& name, std::function<void()> test) {
        tests.push_back([name, test]() {
            current_test_name = name;
            std::cout << "Running test: " << name << "... ";
            try {
                test();
                std::cout << "PASSED" << std::endl;
                passed_tests++;
            } catch (const std::exception& e) {
                std::cout << "FAILED: " << e.what() << std::endl;
                failed_tests++;
            } catch (...) {
                std::cout << "FAILED: Unknown error" << std::endl;
                failed_tests++;
            }
        });
    }
    
    static void run_all_tests() {
        std::cout << "=== HEXAPOD TEST SUITE ===" << std::endl;
        std::cout << "Running " << tests.size() << " tests..." << std::endl;
        
        passed_tests = 0;
        failed_tests = 0;
        
        for (auto& test : tests) {
            test();
        }
        
        std::cout << std::endl;
        std::cout << "Results: " << passed_tests << " PASSED, " << failed_tests << " FAILED" << std::endl;
        
        if (failed_tests == 0) {
            std::cout << "🕷️ ALL TESTS PASSED! Code is ready for upload." << std::endl;
        } else {
            std::cout << "❌ " << failed_tests << " test(s) failed. Fix issues before upload." << std::endl;
        }
    }
    
    static void assert_true(bool condition, const std::string& message = "") {
        if (!condition) {
            throw std::runtime_error("Assertion failed: " + message);
        }
    }
    
    static void assert_equals(int expected, int actual, const std::string& message = "") {
        if (expected != actual) {
            throw std::runtime_error("Expected " + std::to_string(expected) + 
                                   " but got " + std::to_string(actual) + 
                                   (message.empty() ? "" : " - " + message));
        }
    }
    
    static void assert_in_range(int value, int min, int max, const std::string& message = "") {
        if (value < min || value > max) {
            throw std::runtime_error("Value " + std::to_string(value) + 
                                   " not in range [" + std::to_string(min) + 
                                   ", " + std::to_string(max) + "]" +
                                   (message.empty() ? "" : " - " + message));
        }
    }
};

// Определения статических переменных
std::vector<std::function<void()>> TestFramework::tests;
std::string TestFramework::current_test_name;
int TestFramework::passed_tests = 0;
int TestFramework::failed_tests = 0;

// Макрос для регистрации тестов
#define TEST(name) \
    void test_##name(); \
    namespace { \
        struct TestRegistrar_##name { \
            TestRegistrar_##name() { \
                TestFramework::add_test(#name, test_##name); \
            } \
        }; \
        TestRegistrar_##name registrar_##name; \
    } \
    void test_##name()

// Макросы для ассертов
#define ASSERT_TRUE(condition) TestFramework::assert_true(condition, #condition)
#define ASSERT_EQ(expected, actual) TestFramework::assert_equals(expected, actual, #expected " == " #actual)
#define ASSERT_IN_RANGE(value, min, max) TestFramework::assert_in_range(value, min, max, #value " in [" #min ", " #max "]")
