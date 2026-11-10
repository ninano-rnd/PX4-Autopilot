#!/usr/bin/env python3
"""Static regressions for the Goshawk real-flight comparison profile.

Run from any directory with Python's standard library:
    python3 Tools/simulation/jsbsim/tests/test_goshawk_profile.py
"""

from pathlib import Path
import math
import shlex
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
BRIDGE = ROOT / "Tools/simulation/jsbsim/jsbsim_bridge"
MODEL = BRIDGE / "models/goshawk_10s"
AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/34401_jsbsim_goshawk_10s"


def airframe_parameters():
    values = {}
    for line in AIRFRAME.read_text().splitlines():
        fields = shlex.split(line, comments=True)
        if len(fields) >= 4 and fields[:2] in (["param", "set"], ["param", "set-default"]):
            values[fields[2]] = float(fields[3])
    return values


class GoshawkProfileTest(unittest.TestCase):
    def test_xml_syntax(self):
        for path in list(MODEL.rglob("*.xml")) + [BRIDGE / "configs/goshawk_10s.xml"]:
            with self.subTest(path=path):
                ET.parse(path)

    def test_real_flight_gains(self):
        # initial_parameters in log/20261108_flight/07_01_55.ulg
        expected = {"FW_PR_P": 0.15, "FW_RR_P": 0.05, "FW_RR_I": 0.1, "FW_RR_FF": 0.5}
        params = airframe_parameters()
        for name, value in expected.items():
            self.assertAlmostEqual(params[name], value)

    def test_airspeed_input(self):
        config = ET.parse(BRIDGE / "configs/goshawk_10s.xml")
        sensor = config.find("sensors/airspeed")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.findtext("jsb_diff_pressure"), "aero/qbar-psf")
        self.assertGreater(float(sensor.findtext("diff_pressure_stddev")), 0)
        params = airframe_parameters()
        for name in ("SYS_HAS_NUM_ASPD", "FW_USE_AIRSPD", "ASPD_PRIMARY"):
            self.assertEqual(params[name], 1)

    def test_inertia_is_physically_admissible(self):
        mass = ET.parse(MODEL / "mass_goshawk_10s.xml")
        inertia = []
        for axis in ("ixx", "iyy", "izz"):
            element = mass.find(axis)
            self.assertEqual(element.get("unit"), "SLUG*FT2")
            inertia.append(float(element.text))
        self.assertAlmostEqual(inertia[2], 0.799)  # Preserve selected FW roll inertia.
        for i, value in enumerate(inertia):
            self.assertTrue(math.isfinite(value) and value > 0)
            self.assertLess(value, sum(inertia[j] for j in range(3) if j != i))
        for name in ("ixy", "ixz", "iyz"):
            self.assertEqual(float(mass.findtext(name)), 0)
        self.assertAlmostEqual(float(mass.findtext("emptywt")) * 0.45359237, 5.0, places=4)

    def test_reference_geometry(self):
        metrics = ET.parse(MODEL / "metrics_goshawk_10s.xml")
        area = float(metrics.findtext("wingarea")) * 0.09290304
        span = float(metrics.findtext("wingspan")) * 0.3048
        chord = float(metrics.findtext("chord")) * 0.3048
        self.assertAlmostEqual(area, 0.48, places=4)
        self.assertAlmostEqual(span, 1.2, places=4)
        self.assertAlmostEqual(chord, 0.2, places=4)
        self.assertAlmostEqual(span * span / area, 3.0, places=4)

    def test_confirmed_propeller_is_selected(self):
        propulsion = ET.parse(MODEL / "propulsion_goshawk_10s.xml")
        thrusters = propulsion.findall("engine/thruster")
        self.assertEqual(len(thrusters), 4)
        for thruster in thrusters:
            self.assertEqual(thruster.get("file"), "prop_apc_13x8")
        prop = ET.parse(MODEL / "Engines/prop_apc_13x8.xml")
        self.assertEqual(prop.find("diameter").get("unit"), "IN")
        self.assertEqual(float(prop.findtext("diameter")), 13)


if __name__ == "__main__":
    unittest.main()
