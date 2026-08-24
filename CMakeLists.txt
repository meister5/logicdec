cmake_minimum_required(VERSION 3.16)
project(logicdec VERSION 0.1.0 LANGUAGES CXX
        DESCRIPTION "Header-only I2C/SPI/UART protocol decoders for captured digital samples")

# The library itself is headers only, so it carries no build of its own - just
# an include path and the language requirement.
add_library(logicdec INTERFACE)
add_library(logicdec::logicdec ALIAS logicdec)

target_include_directories(logicdec INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(logicdec INTERFACE cxx_std_17)

# Only build tests and tools when logicdec is the project being built, so that
# add_subdirectory() from a firmware project stays quiet.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LOGICDEC_IS_TOP_LEVEL ON)
else()
    set(LOGICDEC_IS_TOP_LEVEL OFF)
endif()

option(LOGICDEC_BUILD_TESTS "Build the logicdec test suite" ${LOGICDEC_IS_TOP_LEVEL})
option(LOGICDEC_BUILD_TOOLS "Build the logicdec command line tool" ${LOGICDEC_IS_TOP_LEVEL})
option(LOGICDEC_BUILD_EXAMPLES "Build the logicdec examples" ${LOGICDEC_IS_TOP_LEVEL})

if(MSVC)
    set(LOGICDEC_WARNINGS /W4)
else()
    set(LOGICDEC_WARNINGS -Wall -Wextra -Wpedantic)
endif()

if(LOGICDEC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(LOGICDEC_BUILD_TOOLS)
    add_executable(logicdec_cli tools/logicdec_cli.cpp)
    target_link_libraries(logicdec_cli PRIVATE logicdec)
    target_compile_options(logicdec_cli PRIVATE ${LOGICDEC_WARNINGS})
    set_target_properties(logicdec_cli PROPERTIES OUTPUT_NAME logicdec)
endif()

if(LOGICDEC_BUILD_EXAMPLES)
    add_executable(hello_i2c examples/hello_i2c.cpp)
    target_link_libraries(hello_i2c PRIVATE logicdec)
    target_compile_options(hello_i2c PRIVATE ${LOGICDEC_WARNINGS})
endif()

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS logicdec EXPORT logicdecTargets)
install(EXPORT logicdecTargets
        FILE logicdecConfig.cmake
        NAMESPACE logicdec::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/logicdec)
