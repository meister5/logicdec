// Smallest useful program: synthesise an I2C transfer, decode it, print it.
//
//   g++ -std=c++17 -I../include hello_i2c.cpp -o hello_i2c

#include <iostream>

#include "logicdec/logicdec.hpp"

using namespace logicdec;

int main() {
    // Pretend this came off a capture buffer instead.
    SampleStream capture = synth::I2c(/*sample_rate=*/1e6, /*clock=*/100e3)
                               .write(0x3C, {0x00, 0xAE})
                               .build();

    I2cResult result = decode_i2c(capture);

    for (const I2cTransfer& transfer : result.transfers) {
        std::cout << (transfer.read ? "read from " : "write to ") << to_hex(transfer.address)
                  << (transfer.address_acked ? "" : " (no ack - is the device there?)") << "\n";
        for (std::uint8_t byte : transfer.data) {
            std::cout << "  " << to_hex(byte) << "\n";
        }
    }

    // Annotations are the same decode, in the form a waveform viewer draws.
    for (const Annotation& annotation : result.annotations) {
        std::cout << capture.time_at(annotation.start) * 1e6 << " us  " << to_string(annotation.kind)
                  << "  " << annotation.label << "\n";
    }

    return result.clean() ? 0 : 1;
}
