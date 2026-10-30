// SPDX-License-Identifier: BSD-3-Clause
// Standalone host tests: g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined
//   src/drivers/rc_input/tests/SBusPulseDecoderTest.cpp -o /tmp/sbus_pulse_test
#include "../SBusPulseDecoder.hpp"
#include "../TimerRCInputArgs.hpp"
#include <array>
#include <cassert>
#include <cstdio>
#include <random>
#include <vector>

using Frame = std::array<uint8_t, 25>;
struct Run { bool high; uint32_t us; };

static std::vector<Run> encode(const Frame &frame, int parity_byte = -1, int stop_byte = -1)
{
	std::vector<Run> runs;
	const auto bit = [&runs](bool high) {
		if (!runs.empty() && runs.back().high == high) {
			runs.back().us += 10;

		} else {
			runs.push_back({high, 10});
		}
	};

	for (unsigned i = 0; i < frame.size(); ++i) {
		bit(true); // inverted start

		for (unsigned b = 0; b < 8; ++b) {
			bit(!(frame[i] & (1u << b)));
		}

		bool parity = __builtin_parity(frame[i]);

		if (int(i) == parity_byte) { parity = !parity; }

		bit(!parity);
		bit(int(i) == stop_byte);
		bit(false);
	}

	assert(!runs.back().high);
	runs.back().us += 4000; // idle before next frame's first rising edge
	return runs;
}

static unsigned feed(SBusPulseDecoder &decoder, std::vector<Run> runs, Frame &out,
		     bool jitter = false, int loss_pair = -1)
{
	unsigned frames = 0;
	uint32_t tail = 0;
	assert(runs.size() % 2 == 0);

	for (unsigned i = 0; i < runs.size(); i += 2) {
		assert(runs[i].high && !runs[i + 1].high);

		if (int(i / 2) == loss_pair) {
			decoder.process_pair(0, 0, out.data(), tail);
			continue;
		}

		if (jitter) {
			runs[i].us += 2;
			runs[i + 1].us -= 2;
		}

		if (decoder.process_pair(runs[i].us, runs[i + 1].us, out.data(), tail)) {
			++frames;
			assert(tail >= 3997 && tail <= 4003);
		}
	}

	return frames;
}

static void sync(SBusPulseDecoder &decoder)
{
	Frame out{};
	uint32_t tail;
	assert(!decoder.process_pair(10, 4000, out.data(), tail));
}

int main()
{
	// Regression: NSH expands rc_input start $RC_INPUT_ARGS with an empty
	// RC_INPUT_ARGS into {"start", ""}, unlike the interactive start command.
	const char *manual[] {"start"};
	const char *boot[] {"start", ""};
	const char *empty[] {"start", "", ""};
	const char *uart[] {"start", "", "-d", "/dev/ttyS4"};
	const char *unknown[] {"start", "unexpected"};
	assert(timer_rc_input_unsupported_arg(1, manual) == nullptr);
	assert(timer_rc_input_unsupported_arg(2, boot) == nullptr);
	assert(timer_rc_input_unsupported_arg(3, empty) == nullptr);
	assert(strcmp(timer_rc_input_unsupported_arg(4, uart), "-d") == 0);
	assert(strcmp(timer_rc_input_unsupported_arg(2, unknown), "unexpected") == 0);

	SBusPulseDecoder decoder;
	Frame in{}, out{};
	in[0] = 0x0f;
	sync(decoder);
	assert(feed(decoder, encode(in), out) == 1 && out == in);

	// Every possible payload byte, followed by random frames, parity and
	// flags (failsafe/frame-loss/digital channels) must survive unchanged.
	for (unsigned v = 0; v < 256; ++v) {
		for (unsigned i = 1; i <= 22; ++i) { in[i] = v; }

		in[23] = v & 15;
		assert(feed(decoder, encode(in), out, true) == 1 && out == in);
	}

	std::mt19937 random(42);

	for (unsigned f = 0; f < 1000; ++f) {
		for (unsigned i = 1; i <= 22; ++i) { in[i] = random(); }

		in[23] = random() & 15;
		in[24] = f % 5 ? uint8_t(0x04 + ((f % 4) << 4)) : 0;
		assert(feed(decoder, encode(in), out) == 1 && out == in);
	}

	// A bad byte invalidates the whole frame, then a real idle reacquires.
	assert(feed(decoder, encode(in, 12), out) == 0);
	assert(decoder.counters().parity_errors > 0);
	assert(feed(decoder, encode(in), out) == 1 && out == in);
	assert(feed(decoder, encode(in, -1, 12), out) == 0);
	assert(decoder.counters().start_stop_errors > 0);
	assert(feed(decoder, encode(in), out) == 1);

	assert(feed(decoder, encode(in), out, false, 10) == 0);
	assert(feed(decoder, encode(in), out) == 1);

	Frame bad = in;
	bad[0] = 0x55;
	assert(feed(decoder, encode(bad), out) == 0);
	assert(decoder.counters().header_rejects > 0);
	bad = in;
	bad[24] = 0x81;
	assert(feed(decoder, encode(bad), out) == 0);
	bad = in;
	bad[23] = 0x80;
	assert(feed(decoder, encode(bad), out) == 0);
	assert(decoder.counters().footer_rejects == 2);
	assert(feed(decoder, encode(in), out) == 1);

	auto short_frame = encode(in);
	short_frame.resize(4); // complete 0x0f header (four physical runs), then idle
	short_frame.back().us += 4000;
	assert(feed(decoder, short_frame, out) == 0);
	assert(decoder.counters().incomplete_frames > 0);
	assert(feed(decoder, encode(in), out) == 1);

	auto glitch = encode(in);
	glitch[8].us = 1;
	assert(feed(decoder, glitch, out) == 0);
	assert(feed(decoder, encode(in), out) == 1);

	// No idle at startup: do not guess a frame boundary from arbitrary bytes.
	decoder.reset();
	assert(feed(decoder, encode(in), out) == 0);
	assert(feed(decoder, encode(in), out) == 1);

	uint32_t high, low;
	assert(sbus_capture_widths(65530, 4, 44, 50, high, low) && high == 10 && low == 40);
	assert(sbus_capture_widths(1234, 1334, 5354, 4120, high, low) && high == 100 && low == 4020);
	assert(!sbus_capture_widths(1234, 1334, 5354, 70000, high, low));
	assert(!sbus_capture_widths(100, 90, 120, 20, high, low));
	assert(!sbus_capture_widths(100, 100, 120, 20, high, low));

	// End-to-end captured widths across repeated 16-bit timer wraps. Frame
	// timestamps must refer to the stop bits, not the next frame's start.
	SBusPulseDecoder wrapped;
	sync(wrapped);
	uint16_t counter = 65000;
	uint64_t time = 1000000;

	for (unsigned f = 0; f < 100; ++f) {
		const uint64_t frame_start = time;
		const auto runs = encode(in);
		unsigned completed = 0;

		for (unsigned i = 0; i < runs.size(); i += 2) {
			const uint16_t falling = counter + runs[i].us;
			const uint16_t rising = falling + runs[i + 1].us;
			const uint32_t elapsed = runs[i].us + runs[i + 1].us;
			assert(sbus_capture_widths(counter, falling, rising, elapsed, high, low));
			counter = rising;
			time += elapsed;
			uint32_t tail = 0;

			if (wrapped.process_pair(high, low, out.data(), tail)) {
				++completed;
				assert(out == in);
				assert(time - tail == frame_start + 3000);
			}
		}

		assert(completed == 1);
	}

	const auto errors = decoder.counters();
	assert(errors.framing_errors == errors.start_stop_errors + errors.header_rejects
	       + errors.footer_rejects + errors.incomplete_frames);
	std::puts("SBusPulseDecoder and TIM3 startup arguments: all tests passed");
}
