// logicdec - command line front end.
//
// Reads a CSV capture (one column per channel, one row per sample) and prints a
// decode. The point is to be able to check a capture from a scope, a cheap
// 8-channel USB analyzer, or a microcontroller's own sampling loop without
// opening a GUI.
//
//     logicdec i2c capture.csv --rate 1000000 --scl 0 --sda 1
//     logicdec uart capture.csv --rate 4000000 --auto-baud
//     logicdec timing capture.csv --rate 1000000 --channel 2
//     logicdec demo

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "logicdec/logicdec.hpp"

using namespace logicdec;

namespace {

struct Args {
    std::string command;
    std::string path;
    double rate = 0.0;
    int channel = 0;
    int scl = 0, sda = 1;
    int clk = 0, mosi = 1, miso = -1, cs = -1;
    int mode = 0, bits = 8;
    bool lsb_first = false;
    double baud = 115200.0;
    bool auto_baud = false;
    int data_bits = 8;
    double stop_bits = 1.0;
    Parity parity = Parity::None;
    bool invert = false;
    bool quiet = false;
};

[[noreturn]] void die(const std::string& message) {
    std::cerr << "logicdec: " << message << "\n";
    std::exit(2);
}

void usage() {
    std::cout <<
        "usage: logicdec <command> [capture.csv] [options]\n"
        "\n"
        "commands:\n"
        "  i2c       decode I2C from an SCL/SDA capture\n"
        "  spi       decode SPI (all four modes)\n"
        "  uart      decode asynchronous serial\n"
        "  timing    frequency, period, duty cycle and edge counts\n"
        "  demo      synthesise traffic and decode it, no capture file needed\n"
        "\n"
        "input:\n"
        "  CSV, one column per channel, one row per sample, values 0 or 1.\n"
        "  Lines beginning with # are ignored, except '# rate: <hz>' which sets\n"
        "  the sample rate. Use - to read standard input.\n"
        "\n"
        "common options:\n"
        "  --rate <hz>        sample rate (required unless the file declares it)\n"
        "  --quiet            summary only, no per-event annotations\n"
        "\n"
        "i2c:      --scl <n> --sda <n>\n"
        "spi:      --clk <n> --mosi <n> --miso <n> --cs <n> --mode <0-3>\n"
        "          --bits <n> --lsb-first\n"
        "uart:     --channel <n> --baud <n> --auto-baud --data-bits <n>\n"
        "          --parity none|even|odd --stop-bits <n> --invert\n"
        "timing:   --channel <n>\n";
}

int int_arg(const std::string& value, const char* flag) {
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        die(std::string("expected a number after ") + flag);
    }
}

double double_arg(const std::string& value, const char* flag) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        die(std::string("expected a number after ") + flag);
    }
}

Args parse_args(int argc, char** argv) {
    Args args;
    if (argc < 2) {
        usage();
        std::exit(1);
    }
    args.command = argv[1];
    if (args.command == "-h" || args.command == "--help" || args.command == "help") {
        usage();
        std::exit(0);
    }

    for (int i = 2; i < argc; ++i) {
        const std::string flag = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                die(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (flag.rfind("--", 0) != 0) {
            if (!args.path.empty()) {
                die("more than one capture file given");
            }
            args.path = flag;
        } else if (flag == "--rate") {
            args.rate = double_arg(next("--rate"), "--rate");
        } else if (flag == "--channel") {
            args.channel = int_arg(next("--channel"), "--channel");
        } else if (flag == "--scl") {
            args.scl = int_arg(next("--scl"), "--scl");
        } else if (flag == "--sda") {
            args.sda = int_arg(next("--sda"), "--sda");
        } else if (flag == "--clk") {
            args.clk = int_arg(next("--clk"), "--clk");
        } else if (flag == "--mosi") {
            args.mosi = int_arg(next("--mosi"), "--mosi");
        } else if (flag == "--miso") {
            args.miso = int_arg(next("--miso"), "--miso");
        } else if (flag == "--cs") {
            args.cs = int_arg(next("--cs"), "--cs");
        } else if (flag == "--mode") {
            args.mode = int_arg(next("--mode"), "--mode");
            if (args.mode < 0 || args.mode > 3) {
                die("SPI mode must be 0, 1, 2 or 3");
            }
        } else if (flag == "--bits") {
            args.bits = int_arg(next("--bits"), "--bits");
        } else if (flag == "--lsb-first") {
            args.lsb_first = true;
        } else if (flag == "--baud") {
            args.baud = double_arg(next("--baud"), "--baud");
        } else if (flag == "--auto-baud") {
            args.auto_baud = true;
        } else if (flag == "--data-bits") {
            args.data_bits = int_arg(next("--data-bits"), "--data-bits");
        } else if (flag == "--stop-bits") {
            args.stop_bits = double_arg(next("--stop-bits"), "--stop-bits");
        } else if (flag == "--invert") {
            args.invert = true;
        } else if (flag == "--parity") {
            const std::string value = next("--parity");
            if (value == "none") {
                args.parity = Parity::None;
            } else if (value == "even") {
                args.parity = Parity::Even;
            } else if (value == "odd") {
                args.parity = Parity::Odd;
            } else {
                die("parity must be none, even or odd");
            }
        } else if (flag == "--quiet") {
            args.quiet = true;
        } else if (flag == "--help") {
            usage();
            std::exit(0);
        } else {
            die("unknown option " + flag);
        }
    }
    return args;
}

// One column per channel, one row per sample. '# rate: <hz>' sets the rate so a
// capture file can be self-describing.
SampleStream read_csv(std::istream& in, double rate_override) {
    std::vector<std::vector<bool>> channels;
    double rate = rate_override;
    std::string line;
    std::size_t row = 0;

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            const std::size_t marker = line.find("rate");
            if (marker != std::string::npos && rate_override <= 0.0) {
                const std::size_t colon = line.find_first_of(":=", marker);
                if (colon != std::string::npos) {
                    try {
                        rate = std::stod(line.substr(colon + 1));
                    } catch (const std::exception&) {
                        // Not a rate declaration after all; treat as a comment.
                    }
                }
            }
            continue;
        }

        std::vector<int> values;
        std::string field;
        std::istringstream fields(line);
        while (std::getline(fields, field, ',')) {
            std::string trimmed;
            for (char c : field) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    trimmed.push_back(c);
                }
            }
            if (trimmed.empty()) {
                continue;
            }
            if (trimmed != "0" && trimmed != "1") {
                if (row == 0) {
                    values.clear();  // a header row
                    break;
                }
                die("row " + std::to_string(row + 1) + ": expected 0 or 1, got '" + trimmed + "'");
            }
            values.push_back(trimmed == "1");
        }

        if (values.empty()) {
            continue;
        }
        if (channels.empty()) {
            channels.resize(values.size());
        } else if (values.size() != channels.size()) {
            die("row " + std::to_string(row + 1) + " has " + std::to_string(values.size()) +
                " channels, expected " + std::to_string(channels.size()));
        }
        for (std::size_t c = 0; c < values.size(); ++c) {
            channels[c].push_back(values[c] != 0);
        }
        ++row;
    }

    if (channels.empty()) {
        die("no samples found in the capture");
    }
    if (rate <= 0.0) {
        die("sample rate unknown: pass --rate, or add '# rate: <hz>' to the file");
    }
    return SampleStream::from_channels(channels, rate);
}

SampleStream load(const Args& args) {
    if (args.path.empty()) {
        die("no capture file given (use - for standard input, or try 'logicdec demo')");
    }
    if (args.path == "-") {
        return read_csv(std::cin, args.rate);
    }
    std::ifstream file(args.path);
    if (!file) {
        die("cannot open " + args.path);
    }
    return read_csv(file, args.rate);
}

std::string timestamp(const SampleStream& stream, SampleIndex index) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << stream.time_at(index) * 1e6 << " us";
    return out.str();
}

void print_annotations(const SampleStream& stream, const std::vector<Annotation>& annotations,
                       bool quiet) {
    if (quiet) {
        return;
    }
    for (const Annotation& annotation : annotations) {
        std::cout << "  " << std::setw(14) << std::right << timestamp(stream, annotation.start)
                  << "  " << std::setw(15) << std::left << to_string(annotation.kind) << " "
                  << annotation.label << "\n";
    }
}

int run_i2c(const Args& args, const SampleStream& stream) {
    I2cConfig config;
    config.scl = args.scl;
    config.sda = args.sda;
    const I2cResult result = decode_i2c(stream, config);

    print_annotations(stream, result.annotations, args.quiet);
    std::cout << "\n" << result.transfers.size() << " transfer(s)\n";
    for (const std::string& line : summarize(result)) {
        std::cout << "  " << line << "\n";
    }
    if (!result.clean()) {
        std::cout << "\n" << result.errors() << " error(s) in the capture\n";
        return 1;
    }
    return 0;
}

int run_spi(const Args& args, const SampleStream& stream) {
    SpiConfig config;
    config.clk = args.clk;
    config.mosi = args.mosi;
    config.miso = args.miso;
    config.cs = args.cs;
    config.bits = args.bits;
    config.cpol = (args.mode & 2) != 0;
    config.cpha = (args.mode & 1) != 0;
    config.msb_first = !args.lsb_first;

    const SpiResult result = decode_spi(stream, config);
    print_annotations(stream, result.annotations, args.quiet);
    std::cout << "\n" << result.words.size() << " word(s) in " << result.transactions
              << " transaction(s), mode " << config.mode() << "\n";
    return result.clean() ? 0 : 1;
}

int run_uart(const Args& args, const SampleStream& stream) {
    UartConfig config;
    config.channel = args.channel;
    config.baud = args.baud;
    config.data_bits = args.data_bits;
    config.parity = args.parity;
    config.stop_bits = args.stop_bits;
    config.invert = args.invert;

    if (args.auto_baud) {
        const double estimate = estimate_baud(stream, args.channel);
        config.baud = nearest_standard_baud(estimate);
        if (config.baud <= 0.0) {
            die("cannot estimate a baud rate: the line never changes level");
        }
        std::cout << "estimated " << static_cast<long>(estimate + 0.5) << " baud, using "
                  << static_cast<long>(config.baud) << "\n\n";
    }

    const UartResult result = decode_uart(stream, config);
    print_annotations(stream, result.annotations, args.quiet);

    std::size_t bad = 0;
    for (const UartFrame& frame : result.frames) {
        bad += frame.ok() ? 0u : 1u;
    }
    std::cout << "\n" << result.frames.size() << " frame(s), " << bad << " with errors\n";
    std::cout << "text: " << result.text() << "\n";
    return bad == 0 ? 0 : 1;
}

int run_timing(const Args& args, const SampleStream& stream) {
    const TimingStats stats = measure_timing(stream, args.channel);
    std::cout << std::fixed;
    std::cout << "channel        " << args.channel << "\n"
              << "samples        " << stream.size() << "\n"
              << std::setprecision(6)
              << "duration       " << stream.duration() * 1e3 << " ms\n"
              << "rising edges   " << stats.rising << "\n"
              << "falling edges  " << stats.falling << "\n"
              << std::setprecision(2)
              << "high fraction  " << high_fraction(stream, args.channel) * 100.0 << " %\n";

    if (!stats.measurable()) {
        std::cout << "\nno complete period in the capture: nothing to measure\n";
        return 0;
    }

    std::cout << std::setprecision(3)
              << "frequency      " << stats.frequency_hz / 1e3 << " kHz\n"
              << "period         " << stats.period_mean_s * 1e6 << " us\n"
              << "period min/max " << stats.period_min_s * 1e6 << " / " << stats.period_max_s * 1e6
              << " us\n"
              << "jitter         " << stats.jitter_s * 1e9 << " ns\n"
              << std::setprecision(2)
              << "duty cycle     " << stats.duty_cycle * 100.0 << " %\n";
    return 0;
}

int run_demo() {
    std::cout << "logicdec " << LOGICDEC_VERSION_STRING << " - synthesised traffic\n\n";

    std::cout << "I2C, 100 kHz: write 0x00 0xAE to 0x3C, then read 0x12 from 0x68\n";
    synth::I2c i2c;
    i2c.write(0x3C, {0x00, 0xAE}).read(0x68, {0x12});
    for (const std::string& line : summarize(decode_i2c(i2c.build()))) {
        std::cout << "  " << line << "\n";
    }

    std::cout << "\nSPI mode 0: 0x9F out, 0xEF back\n";
    SpiConfig spi_config;
    spi_config.clk = 0;
    spi_config.mosi = 1;
    spi_config.miso = 2;
    spi_config.cs = 3;
    const SpiResult spi = decode_spi(synth::Spi(spi_config).transaction({0x9F}, {0xEF}).build(),
                                     spi_config);
    for (const SpiWord& word : spi.words) {
        std::cout << "  mosi " << to_hex(word.mosi) << "  miso " << to_hex(word.miso) << "\n";
    }

    std::cout << "\nUART 115200 8N1\n";
    UartConfig uart_config;
    uart_config.baud = 115200;
    const SampleStream line = synth::Uart(uart_config, 4e6).bytes("hello from logicdec\n").build();
    std::cout << "  " << decode_uart(line, uart_config).text();
    std::cout << "  auto-baud estimate: "
              << static_cast<long>(nearest_standard_baud(estimate_baud(line, 0))) << "\n";

    std::cout << "\nTiming, 10 kHz at 25% duty\n";
    const TimingStats stats = measure_timing(synth::square(1e6, 10e3, 0.25, 50), 0);
    std::cout << std::fixed << std::setprecision(2) << "  " << stats.frequency_hz / 1e3
              << " kHz, duty " << stats.duty_cycle * 100.0 << " %\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);
    try {
        if (args.command == "demo") {
            return run_demo();
        }

        const SampleStream stream = load(args);
        if (args.command == "i2c") {
            return run_i2c(args, stream);
        }
        if (args.command == "spi") {
            return run_spi(args, stream);
        }
        if (args.command == "uart") {
            return run_uart(args, stream);
        }
        if (args.command == "timing") {
            return run_timing(args, stream);
        }
        die("unknown command '" + args.command + "' (try --help)");
    } catch (const std::exception& error) {
        die(error.what());
    }
}
