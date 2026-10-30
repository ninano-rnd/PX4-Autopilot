# Custom Nora: PB4 TIM3 SBUS input

## Implementation

`CONFIG_RC_INPUT_SBUS_CAPTURE=y` selects `src/drivers/rc_input/TimerRCInput.cpp`
instead of the UART `RCInput.cpp`, retaining the `rc_input` shell command and
`input_rc` uORB topic. Other boards retain the original UART implementation.
Only standard inverted SBUS at 100 kbaud / 8E2 is supported. No PPM, DSM binding,
CRSF/GHST or UART RC telemetry is provided by this transport.

Architecture reference inspected locally: ArduPilot `b36fce9fac`,
`libraries/AP_HAL_ChibiOS/SoftSigReaderInt.cpp`, `RCInput.cpp`, and
`libraries/AP_RCProtocol/SoftSerial.cpp`. These GPL sources were not copied.
The new decoder is a separate implementation of inverted UART run-length
reconstruction and SBUS framing. Existing PX4 `sbus_parse()` still unpacks
channels and receiver failsafe/frame-lost flags.

- PB4 AF2, pull-down, TIM3 TI1.
- TIM3 free-running at 1 MHz, ARR=65535, CH1 direct TI1 falling capture,
  CH2 indirect TI1 rising capture, interrupt on CH2 only. No timer reset mode.
- Both input filters use four timer input clock samples (ICxF=2).
- ISR reads hardware capture timestamps and queues HIGH/LOW pairs. No logging,
  allocation, decoding or uORB publishing in the ISR.
- A 512-entry ring is drained on `hp_default` every 1 ms. Overcapture, missing
  edges, ambiguous intervals >=60 ms and ring overflow reset frame assembly;
  no partial frame is passed to the channel decoder.
- Decoder checks bit widths (+/-3 us), start, even parity, both stop bits,
  25-byte length, 0x0f header, SBUS1/SBUS2 footer and reserved flag bits. It
  synchronizes on LOW idle >=300 us. This does not add a CRC to SBUS.
- Paired capture completes the final LOW run at the NEXT rising edge, normally
  the next frame's start. This adds the inter-frame idle to delivery latency.
  `timestamp_last_signal` is reconstructed at the previous frame's end, not
  at delivery time. Frames older than 50 ms are not published. After 100 ms
  without a valid frame, one explicit `rc_lost=true`, `rc_failsafe=true`,
  zero-channel update is published (if reception was previously established).
- Receiver-provided failsafe and frame-lost flags remain distinct. A receiver
  that continues transmitting failsafe frames is not treated as disconnected.

## Timer allocation

| Function | Timer |
|---|---|
| HRT | TIM2 CH3 (was TIM3 CH3) |
| SBUS capture | TIM3 CH1/CH2 |
| UAVCAN clock | TIM6 (was TIM2) |
| PWM outputs | TIM5, TIM4, TIM1, TIM12, unchanged |
| RGB LED | TIM8, unchanged |
| Buzzer | TIM15, unchanged |

TIM2 was NOT free: moving UAVCAN to supported TIM6 is required for this
allocation. Hardware checks must cover HRT and CAN operation as well as RC.
Do not run another TIM3 peripheral driver. UART7 uses its original PF6/PE8
pins and serial-console configuration; PB4 is not assigned to a UART. The
experimental UART diagnostics and NuttX ISR modifications have been removed.
`rc_input status` retains operational counters but no register dumps, pin
polling, or rejected-byte samples. Existing USB and sensor customizations are
retained. The H743XI NuttX target is unchanged;
the H753 target difference is not used as an unproven SBUS fix in this change.

## Build and host decoder tests

```sh
cmake -S src/drivers/rc_input/tests -B /tmp/px4-sbus-tests -DSBUS_SANITIZERS=ON
cmake --build /tmp/px4-sbus-tests
ctest --test-dir /tmp/px4-sbus-tests --output-on-failure
make cuav_nora_default
```

Tests cover all payload byte values, randomized frames, parity/stop/timing
errors, invalid headers/footers/flags, truncated frames, capture loss reset,
reacquisition, timer wrapping, frame-end timestamp reconstruction, and startup
argument handling (interactive start, NSH empty-variable expansion, UART-option
rejection). Framing counter subtotals are checked against their aggregate. They
DO NOT validate MCU pin routing, ISR latency, ring-buffer stress, actual RC
loss timing, HRT continuity or UAVCAN timing; those need the bench tests below.

The NuttX CMake dependency fix for external board headers must be retained.
A successful incremental PX4 link alone is not proof that NuttX pin mappings
were rebuilt. The new module identifies itself as `TIM3 SBUS` in status.

## Bench procedure (propellers removed, disarmed)

Upload `build/cuav_nora_default/cuav_nora_default.px4`. Set in QGC console:

```sh
param set RC_PORT_CONFIG 0
param set RC_INPUT_PROTO 2
reboot
```

`RC_INPUT_PROTO=-1` also enables this fixed SBUS transport; it does not scan
other protocols. `0` disables startup. Other values and `-d` UART arguments
are rejected. Empty arguments from NSH's `$RC_INPUT_ARGS` expansion are accepted
so the standard boot script starts the driver automatically. Restart/reboot
after changing these parameters.

```sh
rc_input status
listener input_rc -n 5
```

Repeat status after 10 seconds. Expected: increasing capture pairs / decoded
frames / Published, 18 (or parser-reported) channels responding correctly.
The capture uses PB4 AF2 and TIM3 PSC=239 with a 240 MHz timer clock; raw GPIO
and timer register dumps are no longer part of the status output. Investigate any
sustained `overcapture`, queue overflow, parity/framing/timing error growth.
The first discontinuity is expected while acquiring the first rising edge.
`Framing details` separates start/stop-bit errors, non-0x0f header bytes,
footer/reserved-flags rejection and incomplete frames at idle. Header rejection
can include non-RC traffic (for example SBUS2 slots), but this must be confirmed
from observations, not assumed. `rc_lost_frame_count` counts the existing PX4
SBUS parser's receiver frame-lost/failsafe indication, not TIM3 overcapture.

Reported bench results confirmed 18-channel reception. The framing subtotal
was attributed to non-0x0f header rejection (last observed byte 0x03), consistent
with SBUS2 non-RC traffic, rather than start/stop-bit or footer errors. The
owner reported completing the requested bench checks before debug cleanup.
Retain the checklist below for regressions after future changes.

Required before flight consideration:

1. Verify every channel, endpoints, switches and receiver failsafe flags.
2. Unplug RC signal: after ~100 ms, `rc_lost`/`rc_failsafe` asserted with
   zero channels; `timestamp_last_signal` must not advance on stale data.
3. Reconnect: valid channels recover after frame synchronization. Test with
   receiver already powered at boot and powered after boot.
4. Exercise `rc_input stop` / `rc_input start`; no IRQ use-after-free or timer
   ownership leak, no stuck capture pin output.
5. Stress CPU, sensor and CAN traffic: no sustained capture/ring overflows,
   no channel corruption; inspect task/work-queue load and RC loss behavior.
6. Verify HRT monotonic timestamps through 65.536 ms counter wrap and long
   runs, normal scheduling and UAVCAN operation after timer reassignment.

This is a new safety-relevant transport, not a flight-qualified fix.
