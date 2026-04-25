param(
    [string]$StreamUrl = "http://192.168.1.145:8080/stream.mjpg",
    [string]$RobotHost = "192.168.1.145",
    [int]$RobotPort = 5005,
    [double]$MaxFps = 5,
    [switch]$Show,
    [switch]$PrintJson
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$Script = Join-Path $RepoRoot "tools/laptop_yolo_sign_bridge.py"

$Args = @(
    $Script,
    "--stream-url", $StreamUrl,
    "--robot-host", $RobotHost,
    "--robot-port", "$RobotPort",
    "--max-fps", "$MaxFps"
)

if ($Show) {
    $Args += "--show"
}

if ($PrintJson) {
    $Args += "--print-json"
}

python @Args

