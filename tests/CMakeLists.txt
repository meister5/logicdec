file(GLOB LOGICDEC_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(logicdec_tests ${LOGICDEC_TEST_SOURCES})
target_link_libraries(logicdec_tests PRIVATE logicdec)
target_include_directories(logicdec_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(logicdec_tests PRIVATE ${LOGICDEC_WARNINGS})

# One ctest entry per suite, so a failure names the protocol rather than just
# "the tests".
foreach(suite SampleStream Edges Timing I2c Spi Uart)
    add_test(NAME ${suite} COMMAND logicdec_tests --filter ${suite}.)
endforeach()

add_test(NAME all COMMAND logicdec_tests)
