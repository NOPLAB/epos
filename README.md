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
uv run --cache-dir .uv-cache python -m unittest discover -s tests
```

The project was initialized using `uv init`. `.venv/` and `.uv-cache/` are local
and ignored by Git. Commit `uv.lock` for reproducible package resolution.

## Commands

```powershell
# Read-only: does not change drive state or parameters.
uv run --cache-dir .uv-cache epos2 inspect

# Halt and disable; verify the drive is stopped before touching the shaft.
uv run --cache-dir .uv-cache epos2 stop

# Explicitly clear only a sole 0x8120 CAN passive-mode warning (USB-only bench).
uv run --cache-dir .uv-cache epos2 clear-can-warning

# Disable and record manual Hall/encoder rotation for 30 seconds.
uv run --cache-dir .uv-cache epos2 hall --seconds 30

# Apply the measured two-pole-pair setting in RAM only.
uv run --cache-dir .uv-cache epos2 correct-pole-pairs

# Ramp to 250 RPM, then hold for 20 seconds.
uv run --cache-dir .uv-cache epos2 run --rpm 250 --hold 20 --confirm-motion

# Ramp to 600 RPM, then hold the command for 30 seconds.
uv run --cache-dir .uv-cache epos2 run --rpm 600 --hold 30 --confirm-motion
```

`--rpm` accepts any nonzero integer from -10,000 to +10,000 RPM.
Negative values request the opposite rotation direction, for example
`epos2 run --rpm -600 --hold 20 --confirm-motion`. The 10,000 RPM magnitude cap
applies to requested speed, device speed limits, and monitored feedback. Zero is rejected;
use `epos2 stop` to stop the drive. Hardware and firmware limits still apply,
and configuration writes must pass read-back verification.

Only positive speeds up to 600 RPM have been physically tested on this assembly.
Reverse rotation and higher speeds are unvalidated; accepting an RPM value does
not establish that the motor, supply, mechanics, or controller can safely achieve it.
The default current cap is 100 mA; fault/overspeed guards remain enabled.
`--current-limit MA` explicitly selects an experimental cap from 1 to 2000 mA.
For example, `epos2 run --rpm 1000 --hold 1 --current-limit 2000 --confirm-motion`.
This is not a motor rating or proof that a particular current is thermally safe.

`--hold` accepts any finite positive duration,
including fractions such as `--hold 0.5`, and defaults to one second. The hold
starts after the acceleration ramp and a measured speed within 10% of the target;
the ramp is not included in the requested duration. Very low speeds may not be
achievable with the existing sensor resolution and bench gains.

`--hold-30` has been replaced by `--hold 30`. The PowerShell wrapper uses
`scripts/run.ps1 -RPM 250 -Hold 20 -ConfirmMotion`.

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

### Restart after power loss

Bench settings are RAM-only and can revert when power is removed. In particular,
the stored pole-pair setting is 1, whereas the measured assembly requires 2.
With EPOS Studio closed, use this sequence after switching power back on:

```powershell
uv run --cache-dir .uv-cache epos2 inspect
# Only if the active warning is the sole 0x8120 CAN warning:
uv run --cache-dir .uv-cache epos2 clear-can-warning
# Restore the measured two pole pairs without enabling or saving to flash:
uv run --cache-dir .uv-cache epos2 correct-pole-pairs
uv run --cache-dir .uv-cache epos2 inspect
```

`run` reports each mismatched motor/sensor value and refuses to enable the drive.
It does not silently repair sensor settings. Once the preflight matches, `run`
reapplies its temporary current limits, gains, commutation, polarity, and profile.
Do not save all startup parameters to flash: the stored 4170/8340 mA current
values and thermal data were not validated as motor ratings.

After a power cycle, run `inspect` first. It includes the error register and
error-history codes. If the drive is stopped and disabled, the register is
exactly `0x10`, and the history contains only `0x8120`, `clear-can-warning` can
reset that warning from Python. It never enables or moves the motor, never
retries, and rejects other or mixed error histories. Verify the command succeeds
before running again. No automatic reset is performed by `run`. Close EPOS Studio
before either command. The vendor's USB-only startup procedure is described in
the [EPOS2 24/5 Getting Started guide](https://www.maxongroup.com/medias/sys_master/8811456954398.pdf).

- Motion is restricted to the measured node 3 configuration: motor type 11,
  two pole pairs, 1000 PPR, sensor type 1, and initial polarity 1.
- The run temporarily selects sinusoidal motor type 10, polarity 0, current
  PI 48/24, velocity PI 300/1, and zero feedforward.
- Both device current-limit objects are set to `--current-limit` (**100 mA** by default). This is an
  experimental cap, **not the motor's rated current**. The 12 V / 2 A bench
  supply limit is not a motor phase-current limit.
- Acceleration/deceleration are 100/200 RPM/s. The monitor aborts for faults,
  lost enable, feedback opposite to the requested direction, current above
  `min(1.5 * current limit, 2000)` mA (150 mA by default; 2000 mA for the
  explicit 2000 mA cap), or absolute speed above `min(abs(target RPM) + 100, 10000)`. Faults are never
  automatically cleared. Device maximum motor/profile speeds are written as
  unsigned magnitudes and never exceed 10,000 RPM; only the velocity command
  carries the requested sign. Parameters are applied when running, not merely
  by editing this project.
- The hold begins after the ramp (`abs(target RPM) / 100` seconds) and a sample
  within 10% of target. It aborts outside 20% of target (also capped at target
  magnitude plus 100 RPM and the absolute 10,000 RPM cap), in the requested direction, if acquisition exceeds
  the ramp plus four seconds, or if
  total motion exceeds that acquisition limit plus the requested hold plus
  two seconds. For 600 RPM and a 30-second hold, these are the previous
  540–660 RPM acquisition band, 480–700 RPM hold band, and 42-second total cap.
  This is not a promise of exact speed or a thermally validated hold duration.
- Cleanup attempts halt and disable independently and verifies zero RPM.
  It resets the target and velocity gains to zero and restores the original
  current gains, motor type, and polarity. Both current limits return to
  100 mA after stop/disable verification. Speed limits, profile mode,
  acceleration, and zero feedforward remain in RAM. Position references may
  change when polarity changes.
- No parameters are saved to nonvolatile memory. A lost USB link, process
  termination, OS failure, or blocked DLL call can defeat software cleanup:
  **cut power if stopping is uncertain**. Timing bounds assume responsive I/O.

## Layout

```text
src/epos2/    Native DLL driver, monitored control, and CLI
scripts/      PowerShell wrappers
tests/        Hardware-free unittest checks
docs/         Bench findings and EPOS Studio project
logs/         Existing measurements and new CSV captures (Git-ignored)
```

Existing logs were preserved in `logs/`, not deleted. New captures use unique
filenames and refuse to overwrite an existing file. `--log PATH` selects a
custom output. To deliberately version a specific measurement, use
`git add -f logs/<filename>`.

Safety-stop messages identify the failed guard and include measured RPM,
current, statusword, and fault state. The triggering sample is also written to
the CSV and console before cleanup; the last successful sample alone cannot
explain a subsequent stop. At 100 RPM/s acceleration, a 10,000 RPM request has
a 100-second ramp before the requested hold can begin.

The former root PowerShell implementations have been replaced by thin wrappers
in `scripts/`. Their old arbitrary gain and high-current run options were
removed; use the Python CLI for the bounded, previously tested configuration.

See [bench findings](docs/bench.md). Licensed under [MIT](LICENSE).
