// A single-header test runner.
//
// logicdec has no dependencies, and a test suite that pulls one in undercuts
// that claim - "header-only, no dependencies" should stay true of the thing you
// clone, not just the thing you ship. This is the ~150 lines needed to register
// tests, compare values, and print a readable failure.

#ifndef LOGICDEC_TEST_FRAMEWORK_HPP
#define LOGICDEC_TEST_FRAMEWORK_HPP

#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace logicdec_test {

struct TestCase {
    std::string suite;
    std::string name;
    void (*fn)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline bool& current_failed() {
    static bool failed = false;
    return failed;
}

struct Registrar {
    Registrar(const char* suite, const char* name, void (*fn)()) {
        registry().push_back({suite, name, fn});
    }
};

// An assertion that cannot continue meaningfully.
struct Abort {};

inline void fail(const char* file, int line, const std::string& message) {
    current_failed() = true;
    std::cout << "\n    " << file << ":" << line << "\n    " << message << "\n";
}

// -- value formatting -------------------------------------------------------

inline std::string describe(bool value) { return value ? "true" : "false"; }

inline std::string describe(std::uint8_t value) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << static_cast<unsigned>(value);
    return out.str();
}

inline std::string describe(const std::string& value) { return "\"" + value + "\""; }
inline std::string describe(const char* value) { return std::string("\"") + value + "\""; }

template <typename T>
std::string describe(const T& value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

template <typename T>
std::string describe(const std::vector<T>& values) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << describe(values[i]);
    }
    out << "] (" << values.size() << ")";
    return out.str();
}

// -- comparison helpers -----------------------------------------------------

template <typename A, typename B>
bool equal(const A& a, const B& b) {
    return a == b;
}

inline bool near(double a, double b, double tolerance) {
    const double diff = a > b ? a - b : b - a;
    return diff <= tolerance;
}

// -- runner -----------------------------------------------------------------

inline int run_all(int argc, char** argv) {
    std::string filter;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--filter") == 0 && i + 1 < argc) {
            filter = argv[++i];
        }
    }

    std::size_t passed = 0;
    std::vector<std::string> failed;

    for (const TestCase& test : registry()) {
        const std::string full = test.suite + "." + test.name;
        if (!filter.empty() && full.find(filter) == std::string::npos) {
            continue;
        }

        current_failed() = false;
        std::cout << "  " << full;
        try {
            test.fn();
        } catch (const Abort&) {
            // already reported
        } catch (const std::exception& error) {
            fail("<exception>", 0, std::string("unexpected exception: ") + error.what());
        } catch (...) {
            fail("<exception>", 0, "unexpected non-standard exception");
        }

        if (current_failed()) {
            failed.push_back(full);
            std::cout << "  FAILED\n";
        } else {
            ++passed;
            std::cout << "  ok\n";
        }
    }

    std::cout << "\n" << passed << " passed, " << failed.size() << " failed\n";
    for (const std::string& name : failed) {
        std::cout << "  failed: " << name << "\n";
    }
    return failed.empty() ? 0 : 1;
}

}  // namespace logicdec_test

#define TEST(suite, name)                                                            \
    static void suite##_##name();                                                    \
    static ::logicdec_test::Registrar suite##_##name##_registrar(#suite, #name,       \
                                                                 suite##_##name);    \
    static void suite##_##name()

#define CHECK(expr)                                                                  \
    do {                                                                             \
        if (!(expr)) {                                                               \
            ::logicdec_test::fail(__FILE__, __LINE__, "expected: " #expr);           \
        }                                                                            \
    } while (false)

#define CHECK_EQ(actual, expected)                                                   \
    do {                                                                             \
        const auto a_ = (actual);                                                    \
        const auto e_ = (expected);                                                  \
        if (!::logicdec_test::equal(a_, e_)) {                                        \
            ::logicdec_test::fail(__FILE__, __LINE__,                                 \
                                  std::string(#actual) + "\n      expected: " +       \
                                      ::logicdec_test::describe(e_) +                 \
                                      "\n      actual:   " +                          \
                                      ::logicdec_test::describe(a_));                 \
        }                                                                            \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                                      \
    do {                                                                             \
        const double a_ = static_cast<double>(actual);                               \
        const double e_ = static_cast<double>(expected);                             \
        if (!::logicdec_test::near(a_, e_, (tolerance))) {                            \
            ::logicdec_test::fail(__FILE__, __LINE__,                                 \
                                  std::string(#actual) + "\n      expected: " +       \
                                      ::logicdec_test::describe(e_) + " +/- " +       \
                                      ::logicdec_test::describe(double(tolerance)) +  \
                                      "\n      actual:   " +                          \
                                      ::logicdec_test::describe(a_));                 \
        }                                                                            \
    } while (false)

// Stops the test on failure. Use when continuing would crash, e.g. before
// indexing into a container whose size was just checked.
#define REQUIRE(expr)                                                                \
    do {                                                                             \
        if (!(expr)) {                                                               \
            ::logicdec_test::fail(__FILE__, __LINE__, "required: " #expr);           \
            throw ::logicdec_test::Abort{};                                          \
        }                                                                            \
    } while (false)

#define CHECK_THROWS(expr)                                                           \
    do {                                                                             \
        bool threw_ = false;                                                         \
        try {                                                                        \
            (void)(expr);                                                            \
        } catch (...) {                                                              \
            threw_ = true;                                                           \
        }                                                                            \
        if (!threw_) {                                                               \
            ::logicdec_test::fail(__FILE__, __LINE__, "expected a throw: " #expr);   \
        }                                                                            \
    } while (false)

#endif  // LOGICDEC_TEST_FRAMEWORK_HPP
