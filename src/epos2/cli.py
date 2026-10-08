"""Command-line entry point. Motion requires an explicit acknowledgement."""

import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

from .control import clear_can_warning, record_hall, require_stopped, run, stop, validate_run
from .device import DEFAULT_DLL, Device


def snapshot(device):
    objects = {
        "motor_type": (0x6402, 0, 2), "pole_pairs": (0x6410, 3, 1),
        "nominal_current_ma": (0x6410, 1, 2), "max_current_ma": (0x6410, 2, 2),
        "max_motor_rpm": (0x6410, 4, 4), "thermal_time_constant": (0x6410, 5, 2),
        "encoder_ppr": (0x2210, 1, 4), "sensor_type": (0x2210, 2, 2),
        "sensor_polarity": (0x2210, 4, 2), "error_register": (0x1001, 0, 1),
        "max_profile_rpm": (0x607F, 0, 4), "hall_pattern": (0x2022, 0, 2),
        "acceleration": (0x6083, 0, 4), "deceleration": (0x6084, 0, 4),
    }
    result = {name: device.read(*address) for name, address in objects.items()}
    result.update(device.sample())
    result["current_pi"] = device.gains("Current")
    result["velocity_pi"] = device.gains("Velocity")
    count = device.read(0x1003, 0, 1)
    result["error_count"] = count
    result["error_history"] = [f"0x{device.read(0x1003, index, 4):08X}" for index in range(1, count + 1)]
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="EPOS2 USB bench control; no permanent parameter saves")
    parser.add_argument("--dll", type=Path, default=DEFAULT_DLL)
    parser.add_argument("--port", default="USB0")
    parser.add_argument("--node", type=int, choices=range(1, 128), default=3, metavar="1..127")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("inspect", help="Read configuration without enabling or disabling the drive")
    commands.add_parser("stop", help="Halt, disable, and verify zero speed")
    commands.add_parser("clear-can-warning", help="Explicitly reset only a sole CAN passive-mode warning (0x8120); never enable the drive")
    commands.add_parser("correct-pole-pairs", help="Set the measured assembly to two pole pairs in RAM")
    hall_parser = commands.add_parser("hall", help="Disable the drive and record manual Hall/encoder motion")
    hall_parser.add_argument("--seconds", type=int, choices=range(1, 121), default=30, metavar="1..120")
    hall_parser.add_argument("--log", type=Path)
    run_parser = commands.add_parser("run", help="Use the previously tested temporary bench configuration")
    run_parser.add_argument("--rpm", type=int, default=600, help="Target RPM from -10000 to 10000, excluding zero; negative values reverse direction")
    run_parser.add_argument("--hold", type=float, default=1, metavar="SECONDS", help="Hold duration after reaching target speed (default: 1 second)")
    run_parser.add_argument("--current-limit", type=int, default=100, metavar="MA", help="Experimental device current cap (1-2000 mA; default: 100)")
    run_parser.add_argument("--confirm-motion", action="store_true", help="Confirm a secured, unloaded motor and accessible power cutoff")
    run_parser.add_argument("--log", type=Path)
    arguments = parser.parse_args(argv)
    if arguments.command == "run":
        if not arguments.confirm_motion:
            parser.error("run requires --confirm-motion; secure the motor and keep clear of the shaft")
        try:
            validate_run(arguments.rpm, arguments.hold, arguments.current_limit)
        except ValueError as error:
            parser.error(str(error))
    if arguments.command in ("run", "correct-pole-pairs", "clear-can-warning") and arguments.node != 3:
        parser.error("The measured bench configuration is restricted to node 3")
    try:
        if arguments.command in ("run", "hall"):
            log = arguments.log or Path("logs") / f"{arguments.command}-{datetime.now():%Y%m%d-%H%M%S-%f}.csv"
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("x", newline="", encoding="utf-8") as output, Device(arguments.dll, arguments.port, arguments.node) as device:
                fields = (["seconds", "rpm", "current_ma", "position_qc", "fault", "statusword", "demand_rpm", "hold_seconds"]
                          if arguments.command == "run" else ["seconds", "position_qc", "hall_pattern"])
                writer = csv.DictWriter(output, fieldnames=fields)
                writer.writeheader()

                def emit(sample):
                    writer.writerow(sample)
                    output.flush()
                    print(json.dumps(sample), flush=True)

                print(f"Log: {log}", flush=True)
                if arguments.command == "run":
                    result = run(device, arguments.rpm, arguments.hold, emit, arguments.current_limit)
                    print(f"Complete: {json.dumps(result)}; stopped and disabled. No permanent save.")
                else:
                    record_hall(device, arguments.seconds, emit)
        else:
            with Device(arguments.dll, arguments.port, arguments.node) as device:
                if arguments.command == "stop":
                    stop(device)
                elif arguments.command == "clear-can-warning":
                    cleared = clear_can_warning(device)
                    print("CAN warning cleared; drive remains disabled." if cleared else "No active fault; nothing cleared.")
                elif arguments.command == "correct-pole-pairs":
                    device.command("SetDisableState")
                    require_stopped(device)
                    if device.read(0x6410, 3, 1) not in (1, 2):
                        raise RuntimeError("Unexpected pole-pair configuration")
                    device.write(0x6410, 3, 2, 1)
                print(json.dumps(snapshot(device), indent=2))
        return 0
    except KeyboardInterrupt:
        print("Interrupted. If stopped/disabled state is uncertain, CUT POWER.", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        return 1
