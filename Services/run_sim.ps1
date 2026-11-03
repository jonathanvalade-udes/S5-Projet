# Lance la pile C++ (SimBridge + FSM) dans WSL2 pour la simulation Godot.
# Usage :  .\Services\run_sim.ps1 [-Telemetry] [-Rebuild] [-Port 9000]
param(
    [switch]$Telemetry,
    [switch]$Rebuild,
    [int]$Port = 9000
)

$servicesDir = $PSScriptRoot
$wslPath = (wsl wslpath -a ($servicesDir -replace '\\', '/')).Trim()

$envVars = "PORT=$Port"
if ($Telemetry) { $envVars += " TELEMETRY=1" }
if ($Rebuild)   { $envVars += " REBUILD=1" }

wsl -e bash -c "$envVars bash '$wslPath/run_sim.sh'"
