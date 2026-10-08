"""Conservative, temporary bench settings for the measured motor assembly."""

import math
import time

MAX_RPM = 10_000
MAX_CURRENT_MA = 2_000


def speed_limit(target_rpm):
    return min(abs(target_rpm) + 100, MAX_RPM)


def validate_run(rpm, hold, current_limit=100):
    if isinstance(rpm, bool) or not isinstance(rpm, int) or rpm == 0 or abs(rpm) > MAX_RPM:
        raise ValueError(f"RPM must be a nonzero integer between -{MAX_RPM} and {MAX_RPM}")
    if isinstance(hold, bool) or not isinstance(hold, (int, float)) or not math.isfinite(hold) or hold <= 0:
        raise ValueError("Hold must be a finite, positive number of seconds")
    if isinstance(current_limit, bool) or not isinstance(current_limit, int) or not 1 <= current_limit <= MAX_CURRENT_MA:
        raise ValueError(f"Current limit must be an integer from 1 to {MAX_CURRENT_MA} mA")


def require_stopped(device):
    if device.read(0x6041, 0, 2) & 0x14 or device.value("GetVelocityIs") != 0:
        raise RuntimeError("Drive must be disabled and stopped")


def clear_can_warning(device):
    require_stopped(device)
    error_register = device.read(0x1001, 0, 1)
    if error_register == 0 and not device.read(0x6041, 0, 2) & 8:
        return False
    count = device.read(0x1003, 0, 1)
    if error_register != 0x10 or count != 1 or device.read(0x1003, 1, 4) & 0xFFFF != 0x8120:
        raise RuntimeError("Refusing reset: expected only CAN warning 0x8120; inspect the device")
    device.command("SetDisableState")
    require_stopped(device)
    try:
        device.command("ClearFault")
    finally:
        device.command("SetDisableState")
        require_stopped(device)
    if device.read(0x1001, 0, 1) or device.read(0x6041, 0, 2) & 8:
        raise RuntimeError("CAN warning remains or returned; do not retry automatically")
    return True


def check_sample(sample, target_rpm, current_limit=100):
    direction = 1 if target_rpm > 0 else -1
    current_trip = min(1.5 * current_limit, MAX_CURRENT_MA)
    reasons = [reason for failed, reason in (
        (sample["fault"] or sample["statusword"] & 8, "device fault"),
        (not sample["statusword"] & 4, "drive disabled"),
        (abs(sample["rpm"]) > speed_limit(target_rpm), f"overspeed (limit {speed_limit(target_rpm)} RPM)"),
        (abs(sample["current_ma"]) > current_trip, f"excess current (limit {current_trip:g} mA)"),
        (direction * sample["rpm"] < -20, "feedback opposite to requested direction"),
    ) if failed]
    if reasons:
        raise RuntimeError(
            f"Safety stop: {', '.join(reasons)}; rpm={sample['rpm']}, "
            f"current_ma={sample['current_ma']}, statusword=0x{sample['statusword']:04X}, fault={sample['fault']}"
        )


def hold_started(elapsed, rpm, target_rpm):
    speed = abs(target_rpm)
    directed_rpm = rpm if target_rpm > 0 else -rpm
    return elapsed >= speed / 100 and 0.9 * speed <= directed_rpm <= 1.1 * speed


def check_hold(elapsed, hold_start, rpm, target_rpm, hold):
    speed = abs(target_rpm)
    directed_rpm = rpm if target_rpm > 0 else -rpm
    acquisition_limit = speed / 100 + 4
    if (elapsed > acquisition_limit + hold + 2
            or (hold_start is None and elapsed > acquisition_limit)
            or (hold_start is not None and not 0.8 * speed <= directed_rpm <= min(1.2 * speed, speed_limit(target_rpm)))):
        raise RuntimeError("Hold speed or time limit exceeded")


def stop(device):
    failures = []
    for command in ("HaltVelocityMovement", "SetDisableState"):
        try:
            device.command(command)
        except Exception as error:
            failures.append(str(error))
    deadline = time.monotonic() + 3
    while True:
        try:
            require_stopped(device)
            break
        except RuntimeError:
            if time.monotonic() >= deadline:
                failures.append("Stopped/disabled state was not confirmed")
                break
            time.sleep(0.05)
    if failures:
        raise RuntimeError("CUT POWER: " + "; ".join(failures))


def run(device, rpm, hold, emit, current_limit=100):
    validate_run(rpm, hold, current_limit)
    speed = abs(rpm)
    direction = 1 if rpm > 0 else -1
    require_stopped(device)
    expected = (("motor_type", 0x6402, 0, 2, 11), ("pole_pairs", 0x6410, 3, 1, 2),
                ("encoder_ppr", 0x2210, 1, 4, 1000), ("sensor_type", 0x2210, 2, 2, 1),
                ("sensor_polarity", 0x2210, 4, 2, 1))
    mismatches = []
    for name, index, subindex, size, value in expected:
        actual = device.read(index, subindex, size)
        if actual != value:
            mismatches.append(f"{name}={actual} (expected {value})")
    if mismatches:
        raise RuntimeError("Unexpected motor/sensor configuration: " + "; ".join(mismatches)
                           + ". Inspect before running; for a reset pole_pairs=1, use 'epos2 correct-pole-pairs'.")
    if device.read(0x1001, 0, 1):
        raise RuntimeError("Device error register is nonzero; faults will not be cleared")
    original_current = device.gains("Current")
    restore = []
    try:
        device.command("SetDisableState")
        restore.append(lambda: device.set_gains("Current", *original_current))
        device.set_gains("Current", 48, 24)
        for index, subindex, size, value in (
            (0x6410, 1, 2, current_limit), (0x6410, 2, 2, current_limit),
            (0x6410, 4, 4, speed_limit(rpm)), (0x607F, 0, 4, speed),
        ):
            device.write(index, subindex, value, size)
        restore.append(lambda: device.write(0x2210, 4, 1, 2))
        device.write(0x2210, 4, 0, 2)
        restore.append(lambda: device.write(0x6402, 0, 11, 2))
        device.write(0x6402, 0, 10, 2)
        device.set_gains("Velocity", 300, 1)
        device.command("SetVelocityRegulatorFeedForward", 0, 0)
        device.command("ActivateProfileVelocityMode")
        device.command("SetVelocityProfile", 100, 200)
        device.command("MoveWithVelocity", 0)
        device.command("SetEnableState")
        if not device.read(0x6041, 0, 2) & 4:
            raise RuntimeError("Operation enable not confirmed")
        initial_position = device.value("GetPositionIs")
        device.command("MoveWithVelocity", rpm)
        started = time.monotonic()
        hold_start = None
        last_sample = None
        while True:
            elapsed = time.monotonic() - started
            if hold_start is not None and elapsed - hold_start >= hold:
                break
            sample = device.sample()
            elapsed = time.monotonic() - started
            sample.update(seconds=elapsed, hold_seconds=0 if hold_start is None else elapsed - hold_start)
            try:
                check_sample(sample, rpm, current_limit)
                check_hold(elapsed, hold_start, sample["rpm"], rpm, hold)
                if hold_start is None and hold_started(elapsed, sample["rpm"], rpm):
                    hold_start = elapsed
                check_hold(elapsed, hold_start, sample["rpm"], rpm, hold)
            except RuntimeError:
                emit(sample)
                raise
            emit(sample)
            last_sample = sample
            time.sleep(0.05)
        if last_sample is None or direction * (last_sample["position_qc"] - initial_position) <= 0:
            raise RuntimeError("No sustained rotation; current limit will not be increased")
        return {"hold_seconds": None if hold_start is None else time.monotonic() - started - hold_start}
    finally:
        stop(device)
        failures = []
        actions = [lambda: device.command("MoveWithVelocity", 0),
                   lambda: device.set_gains("Velocity", 0, 0),
                   lambda: device.write(0x6410, 1, 100, 2),
                   lambda: device.write(0x6410, 2, 100, 2)] + list(reversed(restore))
        for action in actions:
            try:
                action()
            except Exception as error:
                failures.append(str(error))
        if failures:
            raise RuntimeError("Cleanup incomplete; do not re-enable: " + "; ".join(failures))


def record_hall(device, seconds, emit):
    if not 1 <= seconds <= 120:
        raise ValueError("Hall recording must last 1 to 120 seconds")
    device.command("SetDisableState")
    require_stopped(device)
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        if device.read(0x6041, 0, 2) & 0x14:
            device.command("SetDisableState")
            raise RuntimeError("Drive became enabled; cut power and stop hand rotation")
        emit({"seconds": time.monotonic() - started,
              "position_qc": device.value("GetPositionIs"),
              "hall_pattern": device.read(0x2022, 0, 2) & 7})
        time.sleep(0.05)
