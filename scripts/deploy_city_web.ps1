param(
    [string]$Robot = "ubuntu@192.168.1.145",
    [string]$RemoteDir = "/tmp/tb3_city_web_deploy"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$cameraWeb = Join-Path $repo "robot\camera_web"
$rosPkg = Join-Path $repo "src\tb3_camera_web"
$systemd = Join-Path $repo "systemd"

function Run($exe, $arguments) {
    Write-Host ">> $exe $($arguments -join ' ')"
    & $exe @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$exe exited with code $LASTEXITCODE"
    }
}

Run ssh @($Robot, "rm -rf '$RemoteDir' && mkdir -p '$RemoteDir/ros2_package/scripts' '$RemoteDir/ros2_package/launch' '$RemoteDir/ros2_package/resource'")

Run scp @(
    "$cameraWeb\city_map_web.py",
    "$systemd\city-map-web.service",
    "$systemd\web-rviz-state.service",
    "$PSScriptRoot\install_city_web_on_robot.sh",
    "${Robot}:$RemoteDir/"
)

Run scp @(
    "$rosPkg\CMakeLists.txt",
    "$rosPkg\package.xml",
    "${Robot}:$RemoteDir/ros2_package/"
)

Run scp @(
    "$rosPkg\resource\tb3_camera_web",
    "${Robot}:$RemoteDir/ros2_package/resource/"
)

Run scp @(
    "$rosPkg\launch\camera_web.launch.py",
    "$rosPkg\launch\yolo_udp_bridge.launch.py",
    "${Robot}:$RemoteDir/ros2_package/launch/"
)

Run scp @(
    "$rosPkg\scripts\camera_web_node.py",
    "$rosPkg\scripts\yolo_udp_bridge_node.py",
    "$rosPkg\scripts\web_rviz_state_node.py",
    "${Robot}:$RemoteDir/ros2_package/scripts/"
)

Run ssh @("-tt", $Robot, "chmod +x '$RemoteDir/install_city_web_on_robot.sh' && '$RemoteDir/install_city_web_on_robot.sh' '$RemoteDir'")
