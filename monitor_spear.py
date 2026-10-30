import socket

LBF_TO_N = 4.4482216152605

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5763))
sock.settimeout(2.0)

print("Waiting for JSBSim UDP :5763 ...")

try:
	while True:
		try:
			packet, _ = sock.recvfrom(65535)
		except socket.timeout:
			print("Waiting for data ...", flush=True)
			continue

		# JSBSim SOCKET output: time, followed by the listed properties.
		for line in packet.decode(errors="replace").strip().splitlines():
			try:
				values = [float(x.strip()) for x in line.split(",")]
			except ValueError:
				continue  # Skip labels/header packets.

			if len(values) != 17:
				print("Unexpected field count; restart JSBSim with the updated XML.",
					flush=True)
				continue

			(t, alpha, spear_alpha, cl, cd, lift, drag,
			t0, t1, t2, t3, speed, outside, rpm0, rpm1, rpm2, rpm3) = values

			thrust = [x * LBF_TO_N for x in (t0, t1, t2, t3)]
			rpm_text = ", ".join(f"{x:.0f}" for x in (rpm0, rpm1, rpm2, rpm3))

			print("\033[2J\033[H", end="")
			print(f"Simulation time : {t:.2f} s")
			print(f"Airspeed        : {speed * 0.3048:.2f} m/s")
			print(f"AoA JSBSim      : {alpha:.2f} deg")
			print(f"AoA Spear/T022  : {spear_alpha:.2f} deg")
			print(f"CL / CD         : {cl:.5f} / {cd:.5f}")
			print(f"Lift / Drag     : {lift * LBF_TO_N:.3f} / "
				f"{drag * LBF_TO_N:.3f} N")
			print("Motor thrust    : " +
				", ".join(f"{x:.3f}" for x in thrust) + " N")
			print(f"Total thrust    : {sum(thrust):.3f} N")
			print(f"Motor RPM [0–3] : {rpm_text}")
			print(f"Outside T022 0–20 deg: {bool(outside)}", flush=True)
except KeyboardInterrupt:
	pass
finally:
	print()
	sock.close()
