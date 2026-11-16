# logicdec

[![CI](https://github.com/meister5/logicdec/actions/workflows/ci.yml/badge.svg)](https://github.com/meister5/logicdec/actions/workflows/ci.yml)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](CMakeLists.txt)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

Header-only I2C, SPI and UART decoders for captured digital samples, plus timing measurements.

## Features

- I2C: START, repeated START, STOP, 7-bit addresses, R/W, per-byte ACK/NACK
- SPI: all four modes, 1 to 32 bit words, either bit order, chip select in either polarity or free-running
- UART: 5 to 9 data bits, parity, 1 to 2 stop bits, inverted lines, framing and parity errors, baud estimation
- Timing: edge counts, frequency, period, jitter, duty cycle, pulse widths
- Samples are stored packed, one bit per channel, so it fits microcontroller captures
- Waveform generators (`synth`) for testing decoders without a real capture
- A `logicdec` command line tool that reads CSV captures
- No dependencies

## Install

Header-only. Copy or vendor the repository and add it with CMake:

```cmake
add_subdirectory(third_party/logicdec)
target_link_libraries(my_firmware PRIVATE logicdec::logicdec)
```

Or just add `include/` to your include path. Tests, tools and examples are only built when logicdec is the top-level project.

## Usage

```cpp
#include <iostream>
#include "logicdec/logicdec.hpp"

using namespace logicdec;

int main() {
    // Synthesised here; in real use this is your capture buffer.
    SampleStream capture = synth::I2c(/*sample_rate=*/1e6, /*clock=*/100e3)
                               .write(0x3C, {0x00, 0xAE})
                               .build();

    for (const I2cTransfer& transfer : decode_i2c(capture).transfers) {
        std::cout << (transfer.read ? "read from " : "write to ") << to_hex(transfer.address)
                  << (transfer.address_acked ? "" : " (no ack)") << "\n";
    }
}
```

Command line, input is CSV with one column per channel and values 0 or 1:

```bash
logicdec demo                                        # synthesised traffic
logicdec i2c examples/i2c_ssd1306.csv                # rate is read from the file
logicdec uart capture.csv --rate 4000000 --auto-baud
logicdec timing capture.csv --rate 1000000 --channel 2
```

A `# rate: 1000000` comment in the CSV sets the sample rate, and `-` reads standard input. See [`examples/hello_i2c.cpp`](examples/hello_i2c.cpp) for a fuller program. Notes on failure cases (unanswered addresses, truncated captures, partial SPI words) and on testing with generated waveforms are in [`docs/REFERENCE.md`](docs/REFERENCE.md).

Not supported: analog capture, I2C 10-bit address merging, SMBus PEC, I2S, CAN, 1-Wire.

## Development

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## License

MIT. See [LICENSE](LICENSE).
