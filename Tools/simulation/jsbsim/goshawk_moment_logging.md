# Goshawk Pitch moment logging

Run normally:

```sh
make px4_sitl jsbsim_goshawk_10s
```

`sitl_run.sh` prints the CSV path before launching the bridge. For the default
build, each run has a separate directory:

```text
build/px4_sitl_default/rootfs/log/jsbsim/goshawk_<UTC-start-time>_<unique-suffix>/goshawk_moments.csv
```

The logger runs in JSBSim, independently of PX4 arming/ULog and of
`monitor_goshawk.py`. No receiver or extra Python package is needed. Existing
UDP monitor fields are unchanged. The bridge's working directory is set to the
new directory for Goshawk only; model/system paths remain rooted at the bridge
source tree. Launching the bridge directly instead of via `sitl_run.sh` writes
`goshawk_moments.csv` in its working directory and may overwrite that file.

## Data

The model's native CSV output is configured for **50 Hz of simulation time**.
The bridge sets its integration timestep before loading the model, so JSBSim
computes CSV and UDP output intervals using the actual timestep (4 ms).
This also corrects the rates of the existing UDP outputs; packet formats are
unchanged. Logging does not change the physics integration rate.

The first column is JSBSim simulation time in seconds. Subsequent columns have
JSBSim property names in the header. Main diagnostic fields:

| Property | Meaning | Unit |
|---|---|---|
| `moments/m-prop-lbsft` | Total propulsion Pitch moment about CG | lbf ft |
| `moments/m-aero-lbsft` | Total aerodynamic Pitch moment about CG | lbf ft |
| `moments/m-total-lbsft` | Total applied Pitch moment | lbf ft |
| `moments/m-gear-lbsft` | Ground-contact Pitch moment | lbf ft |
| `aero/coefficient/Cmalpha` | Static aerodynamic Pitch moment function output | lbf ft |
| `aero/coefficient/Cmq` | Pitch-rate damping moment function output | lbf ft |
| `velocities/q-rad_sec` | BODY Pitch angular rate | rad/s |
| `accelerations/qdot-rad_sec2` | BODY Pitch angular acceleration | rad/s² |

`Cmalpha` and `Cmq` are **dimensional moment outputs**, despite their coefficient
names. They are already included in the total aerodynamic moment; do not add
them again. `m-aero` also includes force-offset effects between AERORP and CG.
The difference `m-aero - Cmalpha - Cmq` helps inspect those effects in the current
model, but must be revisited if other aerodynamic Pitch functions are added.

BODY Y and the FW Pitch axis coincide in this model. Positive moment/rate is
nose-up. `attitude/phi-deg`, `theta-deg`, `psi-deg` are **raw JSBSim BODY attitude**,
not the adapted FW attitude. The FW attitude is obtained with
`R_FW = R_BODY * Ry(+90 deg)`; simply adding 90 degrees to Euler pitch is not a
general conversion during banked/inverted flight.

Also recorded: BODY roll/yaw rates, air-relative Pitch rate, AoA, sideslip,
dynamic pressure, true airspeed, NED ground velocities, MSL altitude, Lift/Drag,
all four received ESC commands, throttle positions, actual thrusts and RPMs.
Motor order is JSBSim/PX4 indices 0, 1, 2, 3 (not QGC's one-based labels).

Conversions:

- moment: lbf ft × **1.3558179483314** = N m
- force: lbf × **4.4482216152605** = N
- speed: ft/s × **0.3048** = m/s
- horizontal groundspeed: `hypot(v-north-fps, v-east-fps) * 0.3048`
- angular rate/acceleration: rad × **180/pi** = degrees

Do not assume `qdot = m-total / Iyy`: rigid-body rotational coupling can also
contribute. Roll/yaw rates are recorded for that reason.

## Comparison with PX4 ULog

Keep both the CSV and the corresponding `.ulg`. CSV simulation time and ULog
boot timestamps are **not guaranteed to have the same zero**. Align a distinct
motor command change (arming or transition) before comparing timing. The
received `fcs/esc-cmd-norm[*]` fields help with this; account for actuator/engine
response delays when comparing commands to thrust.

Native CSV output can be buffered; inspect the completed file after stopping
the simulation. Abrupt termination can lose the final buffered rows. This is a
separate file, not extra topics inside the `.ulg`.

Start analysis at a point where the controller requests nose-down but actual
Pitch rate increases. Compare propulsion vs aerodynamic vs total Pitch moment,
then commands vs actual thrust. Do not infer physical moment delivery solely
from PX4's `unallocated_torque` or normalized motor commands.
