$ErrorActionPreference = 'Stop'
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$checkDir = Join-Path $env:TEMP ('car-control-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $checkDir | Out-Null
Copy-Item (Join-Path $projectDir '*.h') -Destination $checkDir
Copy-Item (Join-Path $PSScriptRoot 'check.cpp') -Destination $checkDir
# 只在临时副本中注入模拟标定，不修改实际车辆参数；11 不是实车确认值。
$encoder = Get-Content (Join-Path $checkDir 'calibration.h') -Raw
$control = Get-Content (Join-Path $checkDir 'calibration.h') -Raw
$pattern = '(?s)((?:constexpr|const) SteeringCalibrationPoint steeringCalibration\[\] = )\{.*?\};'
$fixture = '$1{{-30.0f,60.0f},{-15.0f,75.0f},{0.0f,90.0f},{15.0f,105.0f},{30.0f,120.0f}};'
[regex]::Replace($control, $pattern, $fixture) | Set-Content (Join-Path $checkDir 'calibration.h') -Encoding utf8
$mockDir = Join-Path $PSScriptRoot 'mocks'
foreach ($mode in @('uncalibrated', 'calibrated')) {
    $value = if ($mode -eq 'uncalibrated') { 'NAN' } else { '11.0f' }
    [regex]::Replace([regex]::Replace($encoder, $pattern, $fixture), 'MOTOR_SHAFT_PULSES_PER_REV = [^;]+;', "MOTOR_SHAFT_PULSES_PER_REV = $value;") | Set-Content (Join-Path $checkDir 'calibration.h') -Encoding utf8
    $flags = @('-std=c++11', '-Wall', '-Wextra', '-pedantic', '-I', $mockDir)
    if ($mode -eq 'uncalibrated') { $flags += '-DUNCALIBRATED' }
    & g++ @flags (Join-Path $checkDir 'check.cpp') -o (Join-Path $checkDir 'check.exe')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & (Join-Path $checkDir 'check.exe')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# 验证非线性、非等距及递减标定表，检查节点、单调性、斜率连续及逆映射。
Copy-Item (Join-Path $PSScriptRoot 'pchip.cpp') -Destination $checkDir
$tables = @(
 '$1{{-30.0f,60.0f},{-15.0f,65.0f},{0.0f,90.0f},{15.0f,100.0f},{30.0f,120.0f}};',
 '$1{{-30.0f,40.0f},{-20.0f,42.0f},{0.0f,90.0f},{10.0f,100.0f},{30.0f,150.0f}};',
 '$1{{-30.0f,150.0f},{-20.0f,148.0f},{0.0f,100.0f},{10.0f,90.0f},{30.0f,40.0f}};'
)
for ($index = 0; $index -lt $tables.Count; ++$index) {
    [regex]::Replace($control, $pattern, $tables[$index]) | Set-Content (Join-Path $checkDir 'calibration.h') -Encoding utf8
    $flags = @('-std=c++11', '-Wall', '-Wextra', '-pedantic', '-I', $mockDir)
    if ($index -eq 0) { $flags += '-DKNOWN_MIDPOINT' }
    & g++ @flags (Join-Path $checkDir 'pchip.cpp') -o (Join-Path $checkDir 'pchip.exe')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & (Join-Path $checkDir 'pchip.exe')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Copy-Item (Join-Path $PSScriptRoot 'hardware.cpp') -Destination $checkDir
& g++ -std=c++11 -Wall -Wextra -pedantic -I $mockDir (Join-Path $checkDir 'hardware.cpp') -o (Join-Path $checkDir 'hardware.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $checkDir 'hardware.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Copy-Item (Join-Path $PSScriptRoot 'fallback.cpp') -Destination $checkDir
& g++ -std=c++11 -Wall -Wextra -pedantic -I $mockDir (Join-Path $checkDir 'fallback.cpp') -o (Join-Path $checkDir 'fallback.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $checkDir 'fallback.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
