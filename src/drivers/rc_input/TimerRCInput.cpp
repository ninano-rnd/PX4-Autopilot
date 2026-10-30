// SPDX-License-Identifier: BSD-3-Clause
/** PB4/TIM3 SBUS transport for the custom Nora board.
 * Architecture reference: ArduPilot RCININT (paired input captures). This
 * implementation uses PX4/NuttX APIs and a separate run-length UART decoder.
 */
#include <board_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <drivers/drv_hrt.h>
#include <lib/rc/sbus.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>
#include <math.h>
#include <inttypes.h>

#include "stm32_gpio.h"
#include "stm32_tim.h"
#include "SBusPulseDecoder.hpp"
#include "TimerRCInputArgs.hpp"

#if !defined(BOARD_SBUS_TIM3_GPIO) || HRT_TIMER == 3
#error "TIM3 SBUS requires a board pin definition and a separate HRT timer"
#endif
#if CONFIG_BOARD_UAVCAN_TIMER_OVERRIDE == 3 || CONFIG_BOARD_UAVCAN_TIMER_OVERRIDE == HRT_TIMER
#error "UAVCAN clock must not share the SBUS or HRT timer"
#endif
#if (STM32_APB1_TIM3_CLKIN % 1000000) != 0
#error "TIM3 clock must divide exactly to 1 MHz"
#endif

namespace
{
constexpr uintptr_t tim = STM32_TIM3_BASE;
constexpr uint32_t overcapture = GTIM_SR_CC1OF | GTIM_SR_CC2OF;
constexpr uint64_t max_frame_age_us = 50000;
constexpr uint64_t signal_timeout_us = 100000;
}

class TimerRCInput : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	TimerRCInput() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}
	~TimerRCInput() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	struct Pair {
		uint64_t timestamp{0}; // rising edge ending the pair, HRT time
		uint32_t high_us{0};
		uint32_t low_us{0};
	};
	struct CaptureStats {
		uint32_t interrupts{0};
		uint32_t pairs{0};
		uint32_t overcaptures{0};
		uint32_t discontinuities{0};
		uint32_t queue_overflows{0};
	};
	static constexpr unsigned QueueSize = 512;
	static constexpr unsigned QueueMask = QueueSize - 1;
	Pair _queue[QueueSize] {};
	unsigned _head{0}; // producer: TIM3 ISR, consumer accesses under IRQ lock
	unsigned _tail{0};
	bool _queue_corrupted{false};
	CaptureStats _capture{};
	bool _started{false};
	bool _have_rising{false};
	uint16_t _previous_rising{0};
	uint64_t _previous_time{0};
	SBusPulseDecoder _decoder{};
	uint32_t _published{0};
	uint32_t _stale_frames{0};
	uint32_t _queue_resets{0};
	uint16_t _lost_frames{0};
	input_rc_s _input{};
	uORB::PublicationMulti<input_rc_s> _publication{ORB_ID(input_rc)};
	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, MODULE_NAME ": TIM3 decode")};

	void Run() override;
	int start_capture();
	static int interrupt(int irq, void *context, void *arg);
	void capture();
	bool pop(Pair &pair);
	void publish_frame(uint8_t *frame, uint64_t timestamp);
};

ModuleBase::Descriptor TimerRCInput::desc{task_spawn, custom_command, print_usage};

TimerRCInput::~TimerRCInput()
{
	ScheduleClear();

	if (_started) {
		irqstate_t flags = px4_enter_critical_section();
		putreg32(0, tim + STM32_GTIM_DIER_OFFSET);
		up_disable_irq(STM32_IRQ_TIM3);
		putreg32(0, tim + STM32_GTIM_CCER_OFFSET);
		putreg32(0, tim + STM32_GTIM_CR1_OFFSET);
		irq_detach(STM32_IRQ_TIM3);
		px4_arch_configgpio(GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTB | GPIO_PIN4);
		px4_leave_critical_section(flags);
	}

	perf_free(_cycle);
}

int TimerRCInput::start_capture()
{
	up_disable_irq(STM32_IRQ_TIM3);
	const int ret = irq_attach(STM32_IRQ_TIM3, interrupt, this);

	if (ret != 0) {
		return ret;
	}

	// TIM3 is dedicated to RC. HRT is on TIM2; PWM outputs use other timers.
	modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM3EN);
	putreg32(0, tim + STM32_GTIM_CR1_OFFSET);
	putreg32(0, tim + STM32_GTIM_DIER_OFFSET);
	putreg32(0, tim + STM32_GTIM_CCER_OFFSET);
	putreg32(0, tim + STM32_GTIM_CR2_OFFSET);
	putreg32(0, tim + STM32_GTIM_SMCR_OFFSET);
	putreg32((STM32_APB1_TIM3_CLKIN / 1000000) - 1, tim + STM32_GTIM_PSC_OFFSET);
	putreg32(0xffff, tim + STM32_GTIM_ARR_OFFSET);
	// Both capture channels observe TI1: CH1 falling, CH2 rising. Four-clock
	// digital input filters reject very short spikes without polling the pin.
	putreg32((1u << GTIM_CCMR1_CC1S_SHIFT) | (2u << GTIM_CCMR1_CC2S_SHIFT)
		 | (2u << GTIM_CCMR1_IC1F_SHIFT) | (2u << GTIM_CCMR1_IC2F_SHIFT), tim + STM32_GTIM_CCMR1_OFFSET);
	putreg32(0, tim + STM32_GTIM_CCMR2_OFFSET);
	putreg32(0, tim + STM32_GTIM_DCR_OFFSET);
	putreg32(GTIM_EGR_UG, tim + STM32_GTIM_EGR_OFFSET);
	putreg32(0, tim + STM32_GTIM_SR_OFFSET);
	const int gpio_ret = px4_arch_configgpio(BOARD_SBUS_TIM3_GPIO);

	if (gpio_ret != PX4_OK) {
		irq_detach(STM32_IRQ_TIM3);
		return gpio_ret;
	}

	putreg32(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC2E, tim + STM32_GTIM_CCER_OFFSET);
	putreg32(GTIM_DIER_CC2IE, tim + STM32_GTIM_DIER_OFFSET);
	_started = true;
	putreg32(GTIM_CR1_CEN, tim + STM32_GTIM_CR1_OFFSET);
	up_enable_irq(STM32_IRQ_TIM3);
#ifdef SPEKTRUM_POWER
	SPEKTRUM_POWER(true);
#endif
	return PX4_OK;
}

int TimerRCInput::interrupt(int irq, void *context, void *arg)
{
	static_cast<TimerRCInput *>(arg)->capture();
	return 0;
}

void TimerRCInput::capture()
{
	const uint32_t status = getreg32(tim + STM32_GTIM_SR_OFFSET);
	// Reading CCRs clears their capture flags. Do not subsequently clear
	// CCxIF in SR: a newer edge might already have arrived.
	const uint16_t falling = getreg32(tim + STM32_GTIM_CCR1_OFFSET);
	const uint16_t rising = getreg32(tim + STM32_GTIM_CCR2_OFFSET);
	const uint32_t errors = (status | getreg32(tim + STM32_GTIM_SR_OFFSET)) & overcapture;
	putreg32(~(errors | GTIM_SR_UIF), tim + STM32_GTIM_SR_OFFSET);
	++_capture.interrupts;

	if (!(status & GTIM_SR_CC2IF)) {
		return;
	}

	const uint64_t now = hrt_absolute_time();
	const uint16_t count = getreg32(tim + STM32_GTIM_CNT_OFFSET);
	const uint64_t edge_time = now - uint16_t(count - rising);
	Pair pair{};
	pair.timestamp = edge_time;

	if (errors != 0) {
		++_capture.overcaptures;

	} else if (_have_rising && (status & GTIM_SR_CC1IF)
		   && sbus_capture_widths(_previous_rising, falling, rising, edge_time - _previous_time,
					  pair.high_us, pair.low_us)) {
		++_capture.pairs;

	} else {
		++_capture.discontinuities;
		pair.high_us = 0;
		pair.low_us = 0;
	}

	// A zero pair is an explicit loss marker; never bridge across a lost edge.
	_previous_rising = rising;
	_previous_time = edge_time;
	_have_rising = true;
	const unsigned next = (_head + 1) & QueueMask;

	if (next == _tail) {
		++_capture.queue_overflows;
		_queue_corrupted = true;

	} else {
		_queue[_head] = pair;
		_head = next;
	}
}

bool TimerRCInput::pop(Pair &pair)
{
	irqstate_t flags = px4_enter_critical_section();

	if (_queue_corrupted) {
		_tail = _head;
		_queue_corrupted = false;
		px4_leave_critical_section(flags);
		_decoder.reset();
		++_queue_resets;
		return false;
	}

	if (_tail == _head) {
		px4_leave_critical_section(flags);
		return false;
	}

	pair = _queue[_tail];
	_tail = (_tail + 1) & QueueMask;
	px4_leave_critical_section(flags);
	return true;
}

void TimerRCInput::publish_frame(uint8_t *frame, uint64_t timestamp)
{
	const uint64_t now = hrt_absolute_time();

	if (timestamp > now || now - timestamp > max_frame_age_us) {
		++_stale_frames;
		return;
	}

	uint16_t count = 0;
	bool failsafe = false;
	bool frame_drop = false;
	unsigned parser_drops = 0;
	uint16_t values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};

	if (!sbus_parse(timestamp, frame, SBusPulseDecoder::FrameSize, values, &count,
			&failsafe, &frame_drop, &parser_drops, input_rc_s::RC_INPUT_MAX_CHANNELS)) {
		return;
	}

	if (frame_drop) {
		++_lost_frames;
	}

	++_published;
	_input.timestamp = now;
	_input.timestamp_last_signal = timestamp;
	_input.channel_count = count;
	memcpy(_input.values, values, sizeof(values));
	_input.rc_failsafe = failsafe;
	_input.rc_lost = false;
	_input.rc_lost_frame_count = _lost_frames;
	_input.rc_total_frame_count = _published;
	_input.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
	_input.rssi = -1;
	_input.link_quality = -1;
	_input.link_snr = -1;
	_input.rssi_dbm = NAN;
	_publication.publish(_input);
}

void TimerRCInput::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	if (!_started && start_capture() != PX4_OK) {
		PX4_ERR("TIM3 capture init failed");
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle);
	Pair pair{};
	uint8_t frame[SBusPulseDecoder::FrameSize] {};

	// Bounded work per invocation even if the input is noisy.
	for (unsigned i = 0; i < QueueSize && pop(pair); ++i) {
		uint32_t tail_us = 0;

		if (_decoder.process_pair(pair.high_us, pair.low_us, frame, tail_us)) {
			publish_frame(frame, pair.timestamp - tail_us);
		}
	}

	const uint64_t now = hrt_absolute_time();

	if (_published != 0 && !_input.rc_lost && now - _input.timestamp_last_signal > signal_timeout_us) {
		_input.timestamp = now;
		_input.rc_lost = true;
		_input.rc_failsafe = true;
		_input.channel_count = 0;
		_publication.publish(_input);
		_decoder.reset();
	}

	perf_end(_cycle);
}

int TimerRCInput::task_spawn(int argc, char *argv[])
{
	// This build is fixed to PB4. NSH may retain the empty $RC_INPUT_ARGS.
	if (const char *arg = timer_rc_input_unsupported_arg(argc, argv)) {
		PX4_ERR("unsupported start argument: '%s'", arg);
		print_usage("timer SBUS does not accept UART options; disable RC_PORT_CONFIG");
		return PX4_ERROR;
	}

	int32_t protocol = -1;

	if (param_get(param_find("RC_INPUT_PROTO"), &protocol) != PX4_OK) {
		PX4_ERR("RC_INPUT_PROTO unavailable");
		return PX4_ERROR;
	}

	if (protocol == 0) {
		return PX4_OK;
	}

	if (protocol != -1 && protocol != 2) {
		print_usage("set RC_INPUT_PROTO to 2 (SBUS) or -1 (auto)");
		return PX4_ERROR;
	}

	auto *instance = new TimerRCInput();

	if (!instance) {
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(1000);
	return PX4_OK;
}

int TimerRCInput::print_status()
{
	irqstate_t flags = px4_enter_critical_section();
	const CaptureStats capture = _capture;
	const unsigned queued = (_head - _tail) & QueueMask;
	px4_leave_critical_section(flags);
	const auto decoded = _decoder.counters();
	PX4_INFO("TIM3 SBUS: PB4 AF2, CH1 falling + CH2 rising, 1 MHz; HRT=TIM%d", HRT_TIMER);
	PX4_INFO("Capture IRQ=%" PRIu32 " pairs=%" PRIu32 " overcapture=%" PRIu32 " discontinuities=%" PRIu32,
		 capture.interrupts, capture.pairs, capture.overcaptures, capture.discontinuities);
	PX4_INFO("Queue=%u overflow=%" PRIu32 " resets=%" PRIu32, queued, capture.queue_overflows, _queue_resets);
	PX4_INFO("8E2 bytes=%" PRIu32 " frames=%" PRIu32 " parity=%" PRIu32 " framing=%" PRIu32 " timing=%" PRIu32,
		 decoded.bytes, decoded.frames, decoded.parity_errors, decoded.framing_errors, decoded.timing_errors);
	PX4_INFO("Framing details: start/stop=%" PRIu32 " header=%" PRIu32 " footer/flags=%" PRIu32 " partial=%" PRIu32,
		 decoded.start_stop_errors, decoded.header_rejects, decoded.footer_rejects, decoded.incomplete_frames);
	PX4_INFO("Published=%" PRIu32 " stale=%" PRIu32 " RC lost=%d failsafe=%d channels=%u",
		 _published, _stale_frames, (int)(_published == 0 || _input.rc_lost), (int)_input.rc_failsafe, _input.channel_count);
	perf_print_counter(_cycle);
	return 0;
}

int TimerRCInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Inverted SBUS on PB4 using TIM3 paired hardware capture. No UART or DSM binding.\n"
				 "RC_INPUT_PROTO: -1/2 enabled, 0 disabled; restart after parameter changes.\n"
				 "Final byte delivery waits for the next rising edge.");
	PRINT_MODULE_USAGE_NAME("rc_input", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rc_input_main(int argc, char *argv[])
{
	return ModuleBase::main(TimerRCInput::desc, argc, argv);
}
