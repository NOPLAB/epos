"""Run with uv run python scripts/check.py; no hardware is loaded."""

import io
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from epos2.cli import main
from epos2.control import check_hold, check_sample, hold_started, run, stop


class FakeDevice:
    def __init__(self):
        self.objects = {(0x6402, 0): 11, (0x6410, 3): 2, (0x2210, 1): 1000,
                        (0x2210, 2): 1, (0x2210, 4): 1, (0x1001, 0): 0}
        self.loops = {"Current": (72, 38), "Velocity": (0, 0)}
        self.enabled = False
        self.target = 0
        self.position = 0
        self.fault = False
        self.commands = []

    def read(self, index, subindex, size):
        if index == 0x6041:
            return 0x0537 if self.enabled else 0x1540
        return self.objects[(index, subindex)]

    def write(self, index, subindex, value, size):
        self.objects[index, subindex] = value

    def gains(self, loop):
        return self.loops[loop]

    def set_gains(self, loop, proportional, integral):
        self.loops[loop] = proportional, integral

    def command(self, name, *arguments):
        self.commands.append(name)
        if name == "SetEnableState":
            self.enabled = True
        elif name == "SetDisableState":
            self.enabled = False
        elif name == "MoveWithVelocity":
            self.target = arguments[0]
        elif name == "HaltVelocityMovement":
            self.target = 0

    def value(self, name):
        return self.position if name == "GetPositionIs" else self.target if self.enabled else 0

    def sample(self):
        self.position += 4000
        return dict(rpm=self.target, current_ma=50, position_qc=self.position,
                    fault=self.fault, statusword=self.read(0x6041, 0, 2), demand_rpm=self.target)


class Checks(unittest.TestCase):
    def test_thresholds(self):
        sample = dict(rpm=600, current_ma=100, fault=False, statusword=0x0537)
        check_sample(sample, 700)
        for field, value in (("rpm", 701), ("rpm", -21), ("current_ma", -151),
                             ("fault", True), ("statusword", 0x1540), ("rpm", -2147483648)):
            with self.assertRaises(RuntimeError):
                check_sample(sample | {field: value}, 700)
        self.assertFalse(hold_started(5.9, 600, 6))
        self.assertTrue(hold_started(6, 600, 6))
        for arguments in ((10.1, None, 600), (43, 6, 600), (7, 6, 479), (7, 6, 701)):
            with self.assertRaises(RuntimeError):
                check_hold(*arguments)

    def test_run_and_cleanup(self):
        device = FakeDevice()
        clock = iter(index / 10 for index in range(2000))
        samples = []
        with patch("epos2.control.time.monotonic", side_effect=lambda: next(clock)), patch("epos2.control.time.sleep"):
            result = run(device, 600, True, samples.append)
        self.assertGreaterEqual(result["hold_seconds"], 30)
        self.assertFalse(device.enabled)
        self.assertEqual(device.target, 0)
        self.assertEqual(device.loops, {"Current": (72, 38), "Velocity": (0, 0)})
        self.assertEqual(device.objects[0x6402, 0], 11)
        self.assertEqual(device.objects[0x2210, 4], 1)
        self.assertEqual(device.objects[0x6410, 2], 100)
        self.assertTrue(samples)

    def test_fault_cleanup(self):
        device = FakeDevice()
        device.fault = True
        with self.assertRaises(RuntimeError):
            run(device, 100, False, lambda sample: None)
        self.assertFalse(device.enabled)
        self.assertEqual(device.target, 0)
        self.assertEqual(device.loops["Velocity"], (0, 0))

    def test_partial_setup_failure_cleanup(self):
        device = FakeDevice()
        original = device.write

        def write(index, subindex, value, size):
            if index == 0x6402 and value == 10:
                original(index, subindex, value, size)
                raise RuntimeError("Injected verification failure")
            original(index, subindex, value, size)

        device.write = write
        with self.assertRaisesRegex(RuntimeError, "Injected verification failure"):
            run(device, 100, False, lambda sample: None)
        self.assertNotIn("SetEnableState", device.commands)
        self.assertFalse(device.enabled)
        self.assertEqual(device.objects[0x6402, 0], 11)
        self.assertEqual(device.objects[0x2210, 4], 1)
        self.assertEqual(device.loops["Current"], (72, 38))

    def test_interrupt_cleanup(self):
        device = FakeDevice()
        with self.assertRaises(KeyboardInterrupt):
            run(device, 100, False, lambda sample: (_ for _ in ()).throw(KeyboardInterrupt()))
        self.assertFalse(device.enabled)
        self.assertEqual(device.loops["Current"], (72, 38))

    def test_disable_even_when_halt_fails(self):
        device = FakeDevice()
        original = device.command

        def command(name, *arguments):
            if name == "HaltVelocityMovement":
                raise RuntimeError("Injected failure")
            original(name, *arguments)

        device.command = command
        with self.assertRaisesRegex(RuntimeError, "CUT POWER"):
            stop(device)
        self.assertIn("SetDisableState", device.commands)

    def test_bad_configuration_never_enables(self):
        device = FakeDevice()
        device.objects[0x6410, 3] = 1
        with self.assertRaises(RuntimeError):
            run(device, 600, True, lambda sample: None)
        self.assertNotIn("SetEnableState", device.commands)

    def test_cli_requires_confirmation_before_loading_dll(self):
        with patch("epos2.cli.Device") as constructor:
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(["run"])
            self.assertEqual(error.exception.code, 2)
            constructor.assert_not_called()

    def test_hold_rejects_invalid_speed_before_loading_dll(self):
        with patch("epos2.cli.Device") as constructor:
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(["run", "--rpm", "100", "--hold-30", "--confirm-motion"])
            constructor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
