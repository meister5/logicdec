# logicdec

[![CI](https://github.com/meister5/logicdec/actions/workflows/ci.yml/badge.svg)](https://github.com/meister5/logicdec/actions/workflows/ci.yml)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](CMakeLists.txt)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

Header-only I2C, SPI and UART decoders I wrote for captured digital samples, plus the timing
measurements underneath them: frequency, period, jitter and duty cycle. No dependencies,
no allocation on the hot path, C++17.

It runs on a desktop against a CSV from a USB analyzer, and on the microcontroller that took
the capture. The second case shaped the design: an ESP32-S3 sampling four channels at 1 MS/s
produces a megabyte a second, so I store samples packed, one bit per channel, rather than one
array per channel.

```cpp
#include "logicdec/logicdec.hpp"
using namespace logicdec;

SampleStream capture(std::move(buffer), /*sample_rate=*/1e6, /*channels=*/2);

for (const I2cTransfer& transfer : decode_i2c(capture).transfers) {
    printf("%s 0x%02X%s\n",
           transfer.read ? "read" : "write",
           transfer.address,
           transfer.address_acked ? "" : "  <- nothing answered");
}
```

## What it decodes

| | |
| --- | --- |
| **I2C** | START / repeated START / STOP, 7-bit addressing, R/W direction, per-byte ACK and NACK, transfers split at repeated starts |
| **SPI** | All four modes, 1–32 bit words, MSB or LSB first, chip select (either polarity) or free-running, MOSI-only / MISO-only / both |
| **UART** | 5–9 data bits, even/odd/no parity, 1–2 stop bits, inverted lines, MSB-first, framing and parity errors, baud estimation |
| **Timing** | Rising/falling counts, frequency, mean/min/max period, jitter, duty cycle, pulse widths |

## Failure cases

A protocol decoder configured slightly wrong does not fail, it produces plausible bytes. Most
of my design decisions follow from that, so I represent the ambiguous cases explicitly instead
of smoothing them over.

**An unanswered address versus an empty write.** If nothing on the bus pulls SDA low, the
transfer carries no data, which looks exactly like a successful write of zero bytes unless the
acknowledge bit is tracked. `I2cTransfer::address_acked` is the difference between "no
device at 0x3C" and "wrote nothing".

**A truncated capture.** `complete` is false when the samples ran
out mid-transfer, and an error annotation says so. A capture that ended early and a device
that stopped clocking are indistinguishable in the bytes alone.

**Partial SPI words are reported rather than padded.** Leftover bits at the end of a chip-select
window almost always mean the mode or word size is wrong. Zero-padding them into a final byte
hides the one clue you had.

**Short glitches are not treated as UART start bits.** A one-sample dip at 1 MS/s is 1/104th
of a 9600-baud bit. Taking it as a start bit yields a byte of noise and desynchronises
everything after it, so I confirm the start bit at its centre before decoding a frame. After a
framing error I resynchronise from the next edge rather than trust the frame length I just
failed to read.

**Too few samples per bit is refused.** Below three samples per bit the bit
centre cannot be placed. `decode_uart` throws rather than returning confident nonsense.

**Timing that cannot be measured says so.** `TimingStats::measurable()` is false with fewer
than two rising edges, instead of a plausible-looking 0 Hz. I measure period rising-edge to
rising-edge, which stays correct at a 10% duty cycle where half-period doubling would report
twice the frequency.

## Testing decoders without a capture

The hard part of testing a protocol decoder is getting waveforms to test it with. Capturing
real traffic pins the tests to whatever the bus was doing that day, and producing a NACK, a
mode mismatch or a corrupted stop bit on demand means deliberately breaking hardware.

So I ship waveform generators, and my tests round-trip through them:

```cpp
SampleStream stream = synth::I2c().write(0x3C, {0x00, 0xAE}).build();
CHECK(decode_i2c(stream).transfers[0].data == std::vector<uint8_t>{0x00, 0xAE});
```

That covers the parameter space rather than one capture: all 128 addresses, all 256 byte
values, all four SPI modes, five baud rates, both bit orders. It also makes the negative tests
possible: `synth::UartDefect{.bad_stop = true}` produces a frame with a broken stop bit, and
decoding a mode-0 SPI capture as mode 1 is asserted *not* to return the right bytes.

The generators are useful outside the tests as well, for exercising a driver's parsing path
with no device attached.

## Building

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

CI builds on GCC, Clang and MSVC across Linux, macOS and Windows, and runs the suite again
under ASan and UBSan.

To use it, add the include directory. That is the whole integration:

```cmake
add_subdirectory(third_party/logicdec)
target_link_libraries(my_firmware PRIVATE logicdec::logicdec)
```

`add_subdirectory` from a firmware project builds nothing: the tests, tools and examples are
only enabled when logicdec is the top-level project.

## Command line

```bash
logicdec demo                                        # synthesised traffic, no capture needed
logicdec i2c examples/i2c_ssd1306.csv                # rate is read from the file
logicdec uart capture.csv --rate 4000000 --auto-baud
logicdec timing capture.csv --rate 1000000 --channel 2
```

Input is CSV: one column per channel, one row per sample, values 0 or 1. A `# rate: 1000000`
comment makes a capture file self-describing. `-` reads standard input, so a capture streamed
off a device over serial can be piped straight in.

```
$ logicdec i2c examples/i2c_ssd1306.csv --quiet

4 transfer(s)
  0x3C W 0x00 0xAE
  0x3C W 0x00 0xD5 0x80
  0x3C W 0x00 0xA8 0x3F
  0x3C W 0x00 0xAF
```

Without `--quiet` you get every annotation with its timestamp (START, address, each byte, each
ACK), which is the transcript view a logic analyzer draws above the waveform.

## Layout

| Header | Contents |
| --- | --- |
| `logicdec/sample_stream.hpp` | Packed capture buffer, channel views, time conversion, `from_patterns` for tests |
| `logicdec/edges.hpp` | Edge finding, pulse widths, `measure_timing` |
| `logicdec/annotation.hpp` | The shared annotation vocabulary every decoder emits |
| `logicdec/i2c.hpp` | `decode_i2c`, `I2cTransfer`, `summarize` |
| `logicdec/spi.hpp` | `decode_spi`, `SpiConfig` (mode, word size, bit order, chip select) |
| `logicdec/uart.hpp` | `decode_uart`, `estimate_baud`, `nearest_standard_baud` |
| `logicdec/synth.hpp` | Waveform generators for I2C, SPI, UART and square waves |

Include `logicdec/logicdec.hpp` for everything, or the individual headers to keep compile
times down on a microcontroller build.

## Writing tests against a waveform you can read

`SampleStream::from_patterns` takes one ASCII row per channel, so a test case can draw the
signal it is about:

```cpp
SampleStream stream = SampleStream::from_patterns({
    "--__--__--__",   // clock
    "-__--__--__-",   // data
}, 1e6);
```

`0`/`1` and `_`/`-` both work; spaces and `|` are ignored so long patterns can be grouped.
`to_patterns()` is the inverse, which makes failure output readable.

## Scope

Digital logic levels only. Analog capture, threshold detection and sample-rate conversion are
out of scope; feed this the ones and zeroes. Not implemented: I2C 10-bit
addressing (both bytes are reported, they are just not merged into one address), SMBus PEC,
and I2S / CAN / 1-Wire.

## License

MIT
