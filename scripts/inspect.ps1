param([switch]$EncoderCheck, [ValidateRange(0, 120)][int]$HallSeconds = 0, [switch]$CorrectPolePairs)
$ErrorActionPreference = 'Stop'
if (@($EncoderCheck, ($HallSeconds -gt 0), $CorrectPolePairs).Where({ $_ }).Count -gt 1) { throw 'Choose only one inspection action' }
$project = Split-Path $PSScriptRoot -Parent
$arguments = @('inspect')
if ($EncoderCheck) { $arguments = @('stop') }
if ($HallSeconds -gt 0) { $arguments = @('hall', '--seconds', "$HallSeconds") }
if ($CorrectPolePairs) { $arguments = @('correct-pole-pairs') }
Push-Location $project
try { & uv run --cache-dir .uv-cache epos2 @arguments; $result = $LASTEXITCODE }
finally { Pop-Location }
exit $result
