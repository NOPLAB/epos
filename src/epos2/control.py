"""Conservative, temporary bench settings for the measured motor assembly."""

import time


def require_stopped(device):
    if device.read(0x6041, 0, 2) & 0x14 or device.value("GetVelocityIs") != 0:
        raise RuntimeError("Drive must be disabled and stopped")


def check_sample(sample, rpm_limit):
    if (sample["fault"] or sample["statusword"] & 8
            or not sample["statusword"] & 4
            or abs(sample["rpm"]) > rpm_limit
            or abs(sample["current_ma"]) > 150 or sample["rpm"] < -20):
        raise RuntimeError("Fault, disabled drive, overspeed, excess current, or reversed feedback")


def hold_started(elapsed, rpm, ramp_seconds):
    return elapsed >= ramp_seconds and 540 <= rpm <= 660


def check_hold(elapsed, hold_start, rpm):
    if (elapsed > 42 or (hold_start is None and elapsed > 10)
            or (hold_start is not None and not 480 <= rpm <= 700)):
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


def run(device, rpm, hold30, emit):
    if rpm not in (100, 300, 600) or (hold30 and rpm != 600):
        raise ValueError("Use 100, 300, or 600 RPM; a 30-second hold requires 600 RPM")
    require_stopped(device)
    expected = ((0x6402, 0, 2, 11), (0x6410, 3, 1, 2),
                (0x2210, 1, 4, 1000), (0x2210, 2, 2, 1), (0x2210, 4, 2, 1))
    if any(device.read(index, subindex, size) != value for index, subindex, size, value in expected):
        raise RuntimeError("Unexpected motor/sensor configuration; inspect before running")
    if device.read(0x1001, 0, 1):
        raise RuntimeError("Device error register is nonzero; faults will not be cleared")
    original_current = device.gains("Current")
    restore = []
    try:
        device.command("SetDisableState")
        restore.append(lambda: device.set_gains("Current", *original_current))
        device.set_gains("Current", 48, 24)
        for index, subindex, size, value in (
            (0x6410, 1, 2, 100), (0x6410, 2, 2, 100),
            (0x6410, 4, 4, rpm + 100), (0x607F, 0, 4, rpm),
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
        duration = rpm / 100 + 1
        last_sample = None
        while True:
            elapsed = time.monotonic() - started
            if (not hold30 and elapsed >= duration) or (
                hold30 and hold_start is not None and elapsed - hold_start >= 30
            ):
                break
            sample = device.sample()
            elapsed = time.monotonic() - started
            check_sample(sample, rpm + 100)
            if hold30:
                if hold_start is None and hold_started(elapsed, sample["rpm"], rpm / 100):
                    hold_start = elapsed
                check_hold(elapsed, hold_start, sample["rpm"])
            sample.update(seconds=elapsed, hold_seconds=0 if hold_start is None else elapsed - hold_start)
            emit(sample)
            last_sample = sample
            time.sleep(0.05)
        if last_sample is None or last_sample["rpm"] < 20 or abs(last_sample["position_qc"] - initial_position) < 100:
            raise RuntimeError("No sustained rotation; current limit will not be increased")
        return {"hold_seconds": None if hold_start is None else time.monotonic() - started - hold_start}
    finally:
        stop(device)
        failures = []
        actions = [lambda: device.command("MoveWithVelocity", 0),
                   lambda: device.set_gains("Velocity", 0, 0)] + list(reversed(restore))
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
