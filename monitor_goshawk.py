"""Display Goshawk JSBSim telemetry from models/goshawk_10s/goshawk_10s.xml.

Run before starting SITL: python3 monitor_goshawk.py
UDP 5764 at 10 Hz; independent of PX4 MAVLink and the Spear monitor (5763).
CL/CD are recovered from the current model's force functions using qbar * S.
At negligible dynamic pressure they cannot be recovered and are shown as N/A.
"""

import math
import socket

LBF_TO_N = 4.4482216152605
FT_TO_M = 0.3048
HOST = "127.0.0.1"
PORT = 5764

# Must match the <output port="5764"> property order; JSBSim prepends time.
FIELDS = (
    "time", "alpha", "lift", "drag", "qbar", "area",
    "thrust0", "thrust1", "thrust2", "thrust3", "speed",
    "rpm0", "rpm1", "rpm2", "rpm3", "roll", "pitch", "yaw",
)


def parse_packet_line(line):
    """Ignore JSBSim headers; reject malformed/non-finite numeric samples."""
    try:
        values = [float(value.strip()) for value in line.split(",")]
    except ValueError:
        return None
    if len(values) != len(FIELDS):
        raise ValueError(
            f"Expected {len(FIELDS)} fields, received {len(values)}; "
            "restart Goshawk SITL with the updated output XML."
        )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Non-finite JSBSim data received; sample not displayed.")
    return dict(zip(FIELDS, values))


def format_sample(sample):
    qs = sample["qbar"] * sample["area"]  # psf * sqft = lbf
    coefficients = (
        f"{sample['lift'] / qs:.5f} / {sample['drag'] / qs:.5f}"
        if qs > 1e-9 else "N/A / N/A (qS approximately zero)"
    )
    thrust = [sample[f"thrust{i}"] * LBF_TO_N for i in range(4)]
    thrust_text = ", ".join(f"{value:.3f}" for value in thrust)
    rpm_text = ", ".join(f"{sample[f'rpm{i}']:.0f}" for i in range(4))
    # Current CFD tables clamp outside this range; this is not a flight limit.
    outside = not -140.0 <= sample["alpha"] <= 0.0
    return "\n".join((
        "Goshawk 10s | JSBSim UDP :5764 | Motors ordered 0, 1, 2, 3",
        f"Simulation time : {sample['time']:.2f} s",
        f"Airspeed        : {sample['speed'] * FT_TO_M:.2f} m/s",
        # Raw JSBSim model attitude, before any PX4 tailsitter sensor rotation.
        f"Euler JSBSim R/P/Y: {sample['roll']:.2f} / "
        f"{sample['pitch']:.2f} / {sample['yaw']:.2f} deg",
        f"AoA JSBSim/FW   : {sample['alpha']:.2f} deg",
        f"CL / CD (L/qS, D/qS): {coefficients}",
        f"Lift / Drag     : {sample['lift'] * LBF_TO_N:.3f} / "
        f"{sample['drag'] * LBF_TO_N:.3f} N",
        f"Motor thrust    : {thrust_text} N",
        f"Total thrust    : {sum(thrust):.3f} N",
        f"Motor RPM [0–3] : {rpm_text}",
        f"Outside CFD -140–0 deg: {outside}",
    ))


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((HOST, PORT))
        sock.settimeout(2.0)
        print(f"Waiting for Goshawk JSBSim UDP {HOST}:{PORT} ...")
        try:
            while True:
                try:
                    packet, _ = sock.recvfrom(65535)
                except socket.timeout:
                    print("Waiting for data (no new sample for 2 s) ...", flush=True)
                    continue

                for line in packet.decode(errors="replace").strip().splitlines():
                    try:
                        sample = parse_packet_line(line)
                    except ValueError as error:
                        print(error, flush=True)
                        continue
                    if sample is not None:
                        print("\033[2J\033[H" + format_sample(sample), flush=True)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
