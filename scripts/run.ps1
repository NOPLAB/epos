param([ValidateSet(100, 300, 600)][int]$RPM = 600, [switch]$Hold30, [switch]$ConfirmMotion)
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$arguments = @('run', '--rpm', "$RPM")
if ($Hold30) { $arguments += '--hold-30' }
if ($ConfirmMotion) { $arguments += '--confirm-motion' }
Push-Location $project
try { & uv run --cache-dir .uv-cache epos2 @arguments; $result = $LASTEXITCODE }
finally { Pop-Location }
exit $result
