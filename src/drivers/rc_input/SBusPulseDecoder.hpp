// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <stdint.h>
#include <string.h>

/** Inverted 100 kbaud, 8E2 SBUS from physical HIGH/LOW runs, in microseconds.
 * No GPIO, timer, allocation or PX4 dependencies. This is not an oversampling
 * UART: timestamps must come from hardware input capture, not GPIO polling.
 */
class SBusPulseDecoder
{
public:
	static constexpr unsigned FrameSize = 25;
	static constexpr uint32_t BitUs = 10;
	static constexpr uint32_t GapUs = 300;

	struct Counters {
		uint32_t bytes{0};
		uint32_t frames{0};
		uint32_t parity_errors{0};
		uint32_t framing_errors{0};
		uint32_t timing_errors{0};
		uint32_t start_stop_errors{0};
		uint32_t header_rejects{0};
		uint32_t footer_rejects{0};
		uint32_t incomplete_frames{0};
	};

	void reset()
	{
		_synced = false;
		_bits = 0;
		_word = 0;
		_count = 0;
	}

	/** One HIGH run followed by one LOW run, ending at the next rising edge.
	 * On success, tail_us is the time from the end of the completed frame to
	 * that next rising edge. It must be subtracted from the capture timestamp.
	 * The final byte is delivered at the next rising edge (normally next frame).
	 */
	bool process_pair(uint32_t high_us, uint32_t low_us, uint8_t *frame, uint32_t &tail_us)
	{
		tail_us = 0;

		if (high_us == 0 || low_us == 0) {
			++_counters.timing_errors;
			reset();
			return false;
		}

		bool complete = false;

		if (_synced) {
			unsigned high_bits = 0;
			unsigned low_bits = 0;

			if (!bit_count(high_us, high_bits) || high_bits > 10
			    || (low_us < GapUs && !bit_count(low_us, low_bits))) {
				++_counters.timing_errors;
				reset();

			} else {
				// Physical HIGH is logical zero. At a byte boundary it is start.
				if (_bits + high_bits > 10) {
					++_counters.start_stop_errors;
					++_counters.framing_errors;
					reset();

				} else {
					_bits += high_bits;
					const unsigned needed = 12 - _bits;
					const unsigned take = (low_us >= GapUs || low_bits >= needed) ? needed : low_bits;
					_word |= ((1u << take) - 1u) << _bits;
					_bits += take;

					if (_bits == 12) {
						if ((_word & 0xc01u) != 0xc00u) {
							++_counters.start_stop_errors;
							++_counters.framing_errors;
							reset();

						} else if (__builtin_parity((_word >> 1) & 0x1ffu) != 0) {
							++_counters.parity_errors;
							reset();

						} else {
							const uint8_t byte = (_word >> 1) & 0xffu;
							++_counters.bytes;
							_bits = 0;
							_word = 0;

							if (_count == 0 && byte != 0x0f) {
								++_counters.header_rejects;
								++_counters.framing_errors;
								reset();

							} else {
								_frame[_count++] = byte;

								if (_count == FrameSize) {
									// SBUS1 footer or one of the four SBUS2 frame footers.
									if ((byte == 0 || (byte & 0xcfu) == 0x04) && (_frame[23] & 0xf0u) == 0) {
										memcpy(frame, _frame, FrameSize);
										const uint32_t used_us = take * BitUs;
										tail_us = low_us > used_us ? low_us - used_us : 0;
										++_counters.frames;
										complete = true;

									} else {
										++_counters.footer_rejects;
										++_counters.framing_errors;
									}

									reset(); // Require a real inter-frame idle before another frame.
								}
							}
						}
					}
				}
			}
		}

		if (low_us >= GapUs) {
			if (_count != 0 || _bits != 0) {
				++_counters.incomplete_frames;
				++_counters.framing_errors;
			}

			reset();
			_synced = true;
		}

		return complete;
	}

	const Counters &counters() const { return _counters; }

private:
	static bool bit_count(uint32_t us, unsigned &bits)
	{
		// Only non-idle runs reach this function. Avoid overflow and reject
		// glitches or widths more than 3 us from an integral 10 us bit count.
		if (us > 120 || us < 7) {
			return false;
		}

		bits = (us + BitUs / 2) / BitUs;
		const uint32_t nominal = bits * BitUs;
		return (us > nominal ? us - nominal : nominal - us) <= 3;
	}

	Counters _counters{};
	bool _synced{false};
	unsigned _bits{0};
	uint16_t _word{0};
	unsigned _count{0};
	uint8_t _frame[FrameSize] {};
};

/** Convert a TIM3 pair to widths, rejecting ambiguous multi-wrap intervals.
 * Falling is CH1, rising is CH2, both mapped to TI1, counter runs at 1 MHz.
 */
inline bool sbus_capture_widths(uint16_t previous_rising, uint16_t falling, uint16_t rising,
				uint64_t elapsed_us, uint32_t &high_us, uint32_t &low_us)
{
	high_us = uint16_t(falling - previous_rising);
	low_us = uint16_t(rising - falling);

	if (elapsed_us >= 60000 || high_us == 0 || low_us == 0) {
		return false;
	}

	// The two widths must cover the captured interval without an extra wrap.
	return high_us + low_us == uint16_t(rising - previous_rising);
}
