# Reference

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

