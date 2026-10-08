"""Hardware-free tests; run with python -m unittest discover -s tests."""

import io
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from epos2.cli import main
from epos2.control import clear_can_warning, check_hold, check_sample, hold_started, run, stop, validate_run


class FakeDevice:
    def __init__(self):
        self.objects = {(0x6402, 0): 11, (0x6410, 3): 2, (0x2210, 1): 1000,
                        (0x2210, 2): 1, (0x2210, 4): 1, (0x1001, 0): 0,
                        (0x1003, 0): 0, (0x1003, 1): 0}
        self.loops = {"Current": (72, 38), "Velocity": (0, 0)}
        self.enabled = False
        self.target = 0
        self.position = 0
        self.fault = False
        self.commands = []

    def read(self, index, subindex, size):
        if index == 0x6041:
            return 0x0537 if self.enabled else 0x1508 if self.fault else 0x1540
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
        elif name == "ClearFault":
            self.fault = False
            self.objects[0x1001, 0] = 0

    def value(self, name):
        return self.position if name == "GetPositionIs" else self.target if self.enabled else 0

    def sample(self):
        self.position += 4000 if self.target > 0 else -4000
        return dict(rpm=self.target, current_ma=50, position_qc=self.position,
                    fault=self.fault, statusword=self.read(0x6041, 0, 2), demand_rpm=self.target)


class Checks(unittest.TestCase):
    def test_thresholds(self):
        sample = dict(rpm=600, current_ma=100, fault=False, statusword=0x0537)
        check_sample(sample, 600)
        for field, value in (("rpm", 701), ("rpm", -21), ("current_ma", -151),
                             ("fault", True), ("statusword", 0x1540), ("rpm", -2147483648)):
            with self.assertRaises(RuntimeError):
                check_sample(sample | {field: value}, 600)
        self.assertFalse(hold_started(5.9, 600, 600))
        self.assertTrue(hold_started(6, 600, 600))
        for arguments in ((10.1, None, 600), (43, 6, 600), (7, 6, 479), (7, 6, 701)):
            with self.assertRaises(RuntimeError):
                check_hold(*arguments, 600, 30)

    def test_run_and_cleanup(self):
        device = FakeDevice()
        clock = iter(index / 10 for index in range(2000))
        samples = []
        with patch("epos2.control.time.monotonic", side_effect=lambda: next(clock)), patch("epos2.control.time.sleep"):
            result = run(device, 600, 30, samples.append)
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
        samples = []
        with self.assertRaisesRegex(RuntimeError, "device fault"):
            run(device, 100, 1, samples.append)
        self.assertEqual(len(samples), 1)
        self.assertTrue(samples[0]["fault"])
        self.assertFalse(device.enabled)
        self.assertEqual(device.target, 0)
        self.assertEqual(device.loops["Velocity"], (0, 0))

    def test_reported_sample_and_current_stop_diagnostics(self):
        sample = dict(rpm=1845, current_ma=109, fault=False, statusword=311)
        check_sample(sample, 10000)
        with self.assertRaisesRegex(RuntimeError, "excess current.*current_ma=151"):
            check_sample(sample | {"current_ma": 151}, 10000)

    def test_current_stop_sample_is_logged(self):
        device = FakeDevice()
        original = device.sample
        device.sample = lambda: original() | {"current_ma": 151}
        samples = []
        with self.assertRaisesRegex(RuntimeError, "excess current"):
            run(device, 10000, 30, samples.append)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["current_ma"], 151)
        self.assertIn("seconds", samples[0])
        self.assertFalse(device.enabled)

    def test_explicit_current_limit(self):
        sample = dict(rpm=1000, current_ma=2000, fault=False, statusword=0x0537)
        check_sample(sample, 1000, 2000)
        for measured in (2001, -2001):
            with self.assertRaisesRegex(RuntimeError, "limit 2000 mA"):
                check_sample(sample | {"current_ma": measured}, 1000, 2000)
        device = FakeDevice()
        clock = iter(index / 100 for index in range(20000))
        samples = []

        def emit(sample):
            self.assertEqual(device.objects[0x6410, 1], 2000)
            self.assertEqual(device.objects[0x6410, 2], 2000)
            samples.append(sample)

        with patch("epos2.control.time.monotonic", side_effect=lambda: next(clock)), patch("epos2.control.time.sleep"):
            run(device, 1000, 0.5, emit, 2000)
        self.assertTrue(samples)
        self.assertEqual(device.objects[0x6410, 1], 100)
        self.assertEqual(device.objects[0x6410, 2], 100)
        self.assertFalse(device.enabled)

    def test_current_limit_validation(self):
        for limit in (0, -1, 2001, True, 1.5):
            device = FakeDevice()
            with self.assertRaises(ValueError):
                run(device, 1000, 1, lambda sample: None, limit)
            self.assertFalse(device.commands)

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
            run(device, 100, 1, lambda sample: None)
        self.assertNotIn("SetEnableState", device.commands)
        self.assertFalse(device.enabled)
        self.assertEqual(device.objects[0x6402, 0], 11)
        self.assertEqual(device.objects[0x2210, 4], 1)
        self.assertEqual(device.loops["Current"], (72, 38))

    def test_interrupt_cleanup(self):
        device = FakeDevice()
        with self.assertRaises(KeyboardInterrupt):
            run(device, 100, 1, lambda sample: (_ for _ in ()).throw(KeyboardInterrupt()))
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
        with self.assertRaisesRegex(RuntimeError, r"pole_pairs=1 \(expected 2\).*correct-pole-pairs"):
            run(device, 600, 30, lambda sample: None)
        self.assertNotIn("SetEnableState", device.commands)

    def test_multiple_configuration_mismatches_reported(self):
        device = FakeDevice()
        device.objects[0x6402, 0] = 10
        device.objects[0x2210, 1] = 500
        with self.assertRaisesRegex(RuntimeError, r"motor_type=10.*encoder_ppr=500"):
            run(device, 600, 1, lambda sample: None)
        self.assertFalse(device.commands)

    def test_cli_requires_confirmation_before_loading_dll(self):
        with patch("epos2.cli.Device") as constructor:
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(["run"])
            self.assertEqual(error.exception.code, 2)
            constructor.assert_not_called()

    def test_invalid_run_arguments_before_loading_dll(self):
        with patch("epos2.cli.Device") as constructor:
            for arguments in (("--rpm", "0"), ("--rpm", "10001"), ("--rpm", "-10001"),
                              ("--hold", "0"), ("--hold", "-1"), ("--hold", "nan"), ("--hold", "inf")):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(["run", *arguments, "--confirm-motion"])
            constructor.assert_not_called()

    def test_custom_rpm_and_hold(self):
        for rpm, hold in ((275, 20), (123, 0.5), (600, 60), (1200, 20), (-275, 20), (-1200, 0.5), (10000, 0.5), (-10000, 0.5)):
            with self.subTest(rpm=rpm, hold=hold):
                device = FakeDevice()
                clock = iter(index / 100 for index in range(20000))
                samples = []
                with patch("epos2.control.time.monotonic", side_effect=lambda: next(clock)), patch("epos2.control.time.sleep"):
                    result = run(device, rpm, hold, samples.append)
                self.assertGreaterEqual(result["hold_seconds"], hold)
                self.assertLess(result["hold_seconds"], hold + 0.1)
                self.assertTrue(all(sample["demand_rpm"] == rpm for sample in samples))
                self.assertFalse(device.enabled)
                self.assertEqual(device.objects[0x607F, 0], abs(rpm))
                self.assertEqual(device.objects[0x6410, 4], min(abs(rpm) + 100, 10000))
                self.assertTrue(all(sample["position_qc"] * rpm > 0 for sample in samples))

    def test_scaled_hold_guards(self):
        self.assertFalse(hold_started(2.74, 275, 275))
        self.assertTrue(hold_started(2.75, 275, 275))
        self.assertFalse(hold_started(3, 200, 275))
        check_hold(10, 3, 275, 275, 20)
        for arguments in ((6.76, None, 275), (29, 3, 275), (4, 3, 219), (4, 3, 331)):
            with self.assertRaises(RuntimeError):
                check_hold(*arguments, 275, 20)

    def test_validation_precedes_device_access(self):
        for rpm, hold in ((0, 20), (10001, 20), (-10001, 20), (2**31, 20), (-(2**31)-1, 20), (275, float("nan")), (275, float("inf")),
                          (275, -1), (True, 20), (275, True), (275, "20")):
            device = FakeDevice()
            with self.assertRaises(ValueError):
                run(device, rpm, hold, lambda sample: None)
            self.assertFalse(device.commands)
        validate_run(1, 0.1)
        validate_run(600, 20)
        validate_run(10000, 20)
        validate_run(-10000, 20)

    def test_custom_cli_arguments(self):
        with tempfile.TemporaryDirectory() as directory, patch("epos2.cli.Device") as constructor, patch("epos2.cli.run", return_value={"hold_seconds": 20}) as runner:
            with redirect_stdout(io.StringIO()):
                result = main(["run", "--rpm", "-1200", "--hold", "20", "--confirm-motion",
                               "--log", str(Path(directory) / "run.csv")])
            self.assertEqual(result, 0)
            self.assertEqual(runner.call_args.args[1:3], (-1200, 20))
            self.assertEqual(runner.call_args.args[0], constructor.return_value.__enter__.return_value)

    def test_reverse_monitoring(self):
        sample = dict(rpm=-600, current_ma=-100, fault=False, statusword=0x0537)
        check_sample(sample, -600)
        for field, value in (("rpm", -701), ("rpm", 21), ("current_ma", 151)):
            with self.assertRaises(RuntimeError):
                check_sample(sample | {field: value}, -600)
        self.assertFalse(hold_started(5.9, -600, -600))
        self.assertTrue(hold_started(6, -600, -600))
        self.assertFalse(hold_started(6, 600, -600))
        check_hold(10, 6, -600, -600, 20)
        for measured in (-479, -701, 600):
            with self.assertRaises(RuntimeError):
                check_hold(10, 6, measured, -600, 20)

    def test_absolute_speed_limit(self):
        for direction in (-1, 1):
            sample = dict(rpm=direction * 10000, current_ma=50, fault=False, statusword=0x0537)
            check_sample(sample, direction * 10000)
            check_hold(101, 100, sample["rpm"], direction * 10000, 20)
            sample["rpm"] = direction * 10001
            with self.assertRaises(RuntimeError):
                check_sample(sample, direction * 10000)
            with self.assertRaises(RuntimeError):
                check_hold(101, 100, sample["rpm"], direction * 10000, 20)

    def test_clear_can_warning(self):
        device = FakeDevice()
        device.fault = True
        device.objects.update({(0x1001, 0): 0x10, (0x1003, 0): 1, (0x1003, 1): 0x8120})
        self.assertTrue(clear_can_warning(device))
        self.assertFalse(device.fault)
        self.assertFalse(device.enabled)
        self.assertEqual(device.commands, ["SetDisableState", "ClearFault", "SetDisableState"])
        device.commands.clear()
        self.assertFalse(clear_can_warning(device))
        self.assertFalse(device.commands)

    def test_unknown_faults_not_cleared(self):
        for register, count, code in ((0x10, 1, 0x8130), (0x10, 2, 0x8120), (0x11, 1, 0x8120), (0x10, 0, 0)):
            device = FakeDevice()
            device.fault = True
            device.objects.update({(0x1001, 0): register, (0x1003, 0): count, (0x1003, 1): code})
            with self.assertRaises(RuntimeError):
                clear_can_warning(device)
            self.assertFalse(device.commands)

    def test_clear_rejects_enabled_or_moving(self):
        device = FakeDevice()
        device.enabled = True
        with self.assertRaises(RuntimeError):
            clear_can_warning(device)
        self.assertFalse(device.commands)
        device.enabled = False
        device.value = lambda name: 10
        with self.assertRaises(RuntimeError):
            clear_can_warning(device)
        self.assertFalse(device.commands)

    def test_failed_clear_still_disables(self):
        device = FakeDevice()
        device.fault = True
        device.objects.update({(0x1001, 0): 0x10, (0x1003, 0): 1, (0x1003, 1): 0x8120})
        original = device.command

        def command(name, *arguments):
            if name == "ClearFault":
                raise RuntimeError("Injected reset failure")
            original(name, *arguments)

        device.command = command
        with self.assertRaisesRegex(RuntimeError, "Injected reset failure"):
            clear_can_warning(device)
        self.assertEqual(device.commands, ["SetDisableState", "SetDisableState"])
        self.assertFalse(device.enabled)


if __name__ == "__main__":
    unittest.main()
