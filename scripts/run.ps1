param([ValidateRange(-10000, 10000)][int]$RPM = 600, [double]$Hold = 1, [ValidateRange(1, 2000)][int]$CurrentLimit = 100, [switch]$ConfirmMotion)
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$arguments = @('run', '--rpm', "$RPM", '--hold', $Hold.ToString([System.Globalization.CultureInfo]::InvariantCulture), '--current-limit', "$CurrentLimit")
if ($ConfirmMotion) { $arguments += '--confirm-motion' }
Push-Location $project
try { & uv run --cache-dir .uv-cache epos2 @arguments; $result = $LASTEXITCODE }
finally { Pop-Location }
exit $result
