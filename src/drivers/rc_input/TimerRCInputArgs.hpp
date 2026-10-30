// SPDX-License-Identifier: BSD-3-Clause
#pragma once

/** argv[0] is "start". NuttX NSH preserves an empty expanded $RC_INPUT_ARGS
 * as an empty argument; accept it, but never silently accept a UART option.
 */
inline const char *timer_rc_input_unsupported_arg(int argc, const char *const argv[])
{
	for (int i = 1; i < argc; ++i) {
		if (argv[i] != nullptr && argv[i][0] != '\0') {
			return argv[i];
		}
	}

	return nullptr;
}
