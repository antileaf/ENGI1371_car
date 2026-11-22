$ErrorActionPreference = 'Stop'
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$checkDir = Join-Path $env:TEMP ('car-control-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $checkDir | Out-Null
Copy-Item (Join-Path $projectDir '*.h') -Destination $checkDir
Copy-Item (Join-Path $PSScriptRoot 'check.cpp') -Destination $checkDir
$calibration = Get-Content (Join-Path $checkDir 'calibration.h') -Raw
$pattern = '(?s)(constexpr SteeringCalibrationPoint steeringCalibration\[\] = )\{.*?\};'
$mockDir = Join-Path $PSScriptRoot 'mocks'
& g++ -std=c++11 -Wall -Wextra -pedantic -I $mockDir (Join-Path $checkDir 'check.cpp') -o (Join-Path $checkDir 'check.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $checkDir 'check.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 验证非线性、非等距及递减标定表，检查节点、单调性、斜率连续及逆映射。
Copy-Item (Join-Path $PSScriptRoot 'pchip.cpp') -Destination $checkDir
$tables = @(
 '$1{{-30.0f,60.0f},{-15.0f,65.0f},{0.0f,90.0f},{15.0f,100.0f},{30.0f,120.0f}};',
 '$1{{-30.0f,40.0f},{-20.0f,42.0f},{0.0f,90.0f},{10.0f,100.0f},{30.0f,150.0f}};',
 '$1{{-30.0f,150.0f},{-20.0f,148.0f},{0.0f,100.0f},{10.0f,90.0f},{30.0f,40.0f}};'
)
for ($index = 0; $index -lt $tables.Count; ++$index) {
    [regex]::Replace($calibration, $pattern, $tables[$index]) | Set-Content (Join-Path $checkDir 'calibration.h') -Encoding utf8
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

# 使用真实主循环验证初始化重试；只替换 Flash 日志和关闭 Wi-Fi。
Copy-Item (Join-Path $projectDir 'calibration.h') -Destination $checkDir
Copy-Item (Join-Path $PSScriptRoot 'edge.cpp') -Destination $checkDir
& g++ -std=c++11 -Wall -Wextra -I $mockDir (Join-Path $checkDir 'edge.cpp') -o (Join-Path $checkDir 'edge.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $checkDir 'edge.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
'#pragma once
namespace boot_log { inline void begin() {} inline void printRecords() {} }' | Set-Content (Join-Path $checkDir 'boot_log.h') -Encoding utf8
$sketch = Get-Content (Join-Path $projectDir 'main.ino') -Raw
$sketch = $sketch.Replace('#define ENABLE_WEB_DEBUG 1', '#define ENABLE_WEB_DEBUG 0')
$sketch = $sketch.Replace('constexpr bool hardware_test_mode = true;', 'constexpr bool hardware_test_mode = false;')
$sketch | Set-Content (Join-Path $checkDir 'sketch.h') -Encoding utf8
Copy-Item (Join-Path $PSScriptRoot 'initialization.cpp') -Destination $checkDir
& g++ -std=c++17 -Wall -Wextra -I $mockDir (Join-Path $checkDir 'initialization.cpp') -o (Join-Path $checkDir 'initialization.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $checkDir 'initialization.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
