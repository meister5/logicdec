// logicdec - umbrella header.
//
// Header-only, C++17, no dependencies. Include this to get everything, or the
// individual headers to keep compile times down on a microcontroller build.

#ifndef LOGICDEC_LOGICDEC_HPP
#define LOGICDEC_LOGICDEC_HPP

#include "logicdec/annotation.hpp"
#include "logicdec/edges.hpp"
#include "logicdec/i2c.hpp"
#include "logicdec/sample_stream.hpp"
#include "logicdec/spi.hpp"
#include "logicdec/synth.hpp"
#include "logicdec/uart.hpp"

#define LOGICDEC_VERSION_MAJOR 0
#define LOGICDEC_VERSION_MINOR 1
#define LOGICDEC_VERSION_PATCH 0
#define LOGICDEC_VERSION_STRING "0.1.0"

#endif  // LOGICDEC_LOGICDEC_HPP
