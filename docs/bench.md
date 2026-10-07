# Bench Findings — 2026-10-07

## Equipment

- Drive: maxon EPOS2 24/5, USB0, node 3.
- Motor labels: EC-powermax 326885 and 326911; markings 877178 and 881868.
- Power supply: 12 V with a 2 A supply-side limit.
- No manufacturer motor datasheet was available. Rated current, resistance,
  inductance, torque constant, and thermal data remain unidentified.

## Sensor measurements

Ten clockwise manual revolutions produced 40,021 encoder counts, approximately
4002.1 counts/revolution. The selected 1000 PPR encoder setting corresponds to
4000 quadrature counts/revolution. Hall transitions repeated approximately
every 2000 counts, supporting two pole pairs. Clockwise counts were positive
with the original encoder polarity 1; the successful powered configuration
temporarily used polarity 0. Do not compare absolute positions across polarity
changes or drive enable transitions.

## Successful powered experiment

EPOS Studio's current-loop identification produced 192/97; a softer candidate
was 48/24. The latter, with velocity PI 300/1 and sinusoidal commutation,
was used for the successful low-current experiments. These are bench candidates,
not manufacturer-recommended gains or complete motor identification.

The original PowerShell 600 RPM hold completed in 30.034 seconds after its ramp.
495 logged hold samples had mean 599.485 RPM, minimum 525 RPM, maximum 690 RPM,
and maximum absolute measured current 79 mA. Device current limits were 100 mA.
Afterward, statusword 0x1540 and zero RPM confirmed disable and stop.

Original evidence is preserved locally in `logs/run-600rpm-30s.log`,
`logs/hall-20261007-150943.csv`, and the trial logs. These files are Git-ignored
by default. `EPOS2 Project.pjm` is a PC-side Studio project, not proof that the
drive's flash parameters contain the successful configuration.

## Refactoring validation

The Python port is checked with a fake device and clock without moving the
motor. A read-only native DLL inspection also succeeded: node 3 reported zero
RPM, statusword 0x1540, no fault, and the expected retained bench settings.
The historical hardware result above belongs to the PowerShell version;
it must not be represented as a powered validation of the Python port.

Native function signatures were checked against the locally installed maxon
`Definitions.h`. DWORD/BOOL/long are 32-bit, WORD is 16-bit, and handles match
the 64-bit Python process. The wrapper uses Windows stdcall through `WinDLL`.
