# EPOS2 Bench Control

Python tools for the maxon **EPOS2 24/5**, connected over USB on Windows.
The code lives in `src/epos2/`; future EPOS4 support should remain separate.
The driver uses the installed maxon command DLL directly through `ctypes`,
with no Python runtime dependencies.

## Setup

Install 64-bit Python 3.12 or later, uv, the maxon USB driver, and the EPOS2
command library. Close EPOS Studio before opening the USB device.

```powershell
cd C:\Users\nop\dev\lab\epos2
uv sync --cache-dir .uv-cache
uv run --cache-dir .uv-cache epos2 --help
uv run --cache-dir .uv-cache python scripts/check.py
```

The project was initialized using `uv init`. `.venv/` and `.uv-cache/` are local
and ignored by Git. Commit `uv.lock` for reproducible package resolution.

## Commands

```powershell
# Read-only: does not change drive state or parameters.
uv run --cache-dir .uv-cache epos2 inspect

# Halt and disable; verify the drive is stopped before touching the shaft.
uv run --cache-dir .uv-cache epos2 stop

# Disable and record manual Hall/encoder rotation for 30 seconds.
uv run --cache-dir .uv-cache epos2 hall --seconds 30

# Apply the measured two-pole-pair setting in RAM only.
uv run --cache-dir .uv-cache epos2 correct-pole-pairs

# Short 100 RPM trial (approximately two seconds including ramp).
uv run --cache-dir .uv-cache epos2 run --rpm 100 --confirm-motion

# Ramp to 600 RPM, then hold the command for 30 seconds.
uv run --cache-dir .uv-cache epos2 run --rpm 600 --hold-30 --confirm-motion
```

`--confirm-motion` acknowledges that the motor is secured, unloaded, the shaft
is clear, and an immediate physical power cutoff is accessible. **Do not run
this configuration on an unidentified assembly or under load.** The software
is a supervised bench experiment, not a production or safety-rated controller.

Global options precede the command, for example:

```powershell
uv run --cache-dir .uv-cache epos2 --dll "C:\path\EposCmd64.dll" --port USB0 --node 3 inspect
```

Default DLL path: the installed EPOS2 C# example's `EposCmd64.dll` under
`C:\Program Files (x86)\maxon motor ag\EPOS IDX\EPOS2\04 Programming\Windows DLL`.
Use a 64-bit DLL and its vendor-supplied dependencies. Vendor binaries are not
included or covered by this project's MIT license.

## Safety and cleanup

- Motion is restricted to the measured node 3 configuration: motor type 11,
  two pole pairs, 1000 PPR, sensor type 1, and initial polarity 1.
- The run temporarily selects sinusoidal motor type 10, polarity 0, current
  PI 48/24, velocity PI 300/1, and zero feedforward.
- Both device current-limit objects are set to **100 mA**. This is an
  experimental cap, **not the motor's rated current**. The 12 V / 2 A bench
  supply limit is not a motor phase-current limit.
- Acceleration/deceleration are 100/200 RPM/s. The monitor aborts for faults,
  lost enable, reversed feedback, current above 150 mA, or speed above target
  plus 100 RPM. Faults are never automatically cleared.
- The 600 RPM hold begins after the six-second ramp and a 540–660 RPM sample.
  It aborts outside 480–700 RPM, if acquisition exceeds ten seconds, or if
  total motion exceeds 42 seconds. This is not a promise of exact speed.
- Cleanup attempts halt and disable independently and verifies zero RPM.
  It resets the target and velocity gains to zero and restores the original
  current gains, motor type, and polarity. Current/speed limits, profile mode,
  acceleration, and zero feedforward remain in RAM. Position references may
  change when polarity changes.
- No parameters are saved to nonvolatile memory. A lost USB link, process
  termination, OS failure, or blocked DLL call can defeat software cleanup:
  **cut power if stopping is uncertain**. Timing bounds assume responsive I/O.

## Layout

```text
src/epos2/    Native DLL driver, monitored control, and CLI
scripts/      PowerShell wrappers and hardware-free checks
docs/         Bench findings and EPOS Studio project
logs/         Existing measurements and new CSV captures (Git-ignored)
```

Existing logs were preserved in `logs/`, not deleted. New captures use unique
filenames and refuse to overwrite an existing file. `--log PATH` selects a
custom output. To deliberately version a specific measurement, use
`git add -f logs/<filename>`.

The former root PowerShell implementations have been replaced by thin wrappers
in `scripts/`. Their old arbitrary gain and high-current run options were
removed; use the Python CLI for the bounded, previously tested configuration.

See [bench findings](docs/bench.md). Licensed under [MIT](LICENSE).
