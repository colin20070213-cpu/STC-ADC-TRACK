$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $gcc = (Get-Command gcc -ErrorAction Stop).Source
    # 原库 LCD_PrintFloat 的有符号比较在主机 GCC 下告警，与本次显示寻址无关。
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-Wno-sign-compare' '-Itests/lcd_mocks' '-I../../Libraries/boards/inc' 'tests/test_lcd.c' '../../Libraries/boards/src/LCD.c' '../../Libraries/boards/src/Font.c' '-o' 'MDK/test_lcd.exe'
    if ($LASTEXITCODE -ne 0) { throw 'OLED bus test compilation failed' }
    & '.\MDK\test_lcd.exe'
    if ($LASTEXITCODE -ne 0) { throw 'OLED bus tests failed' }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' '-DTRACK_IAP_REGISTER_TEST' '-IUSER/inc' '-Itests' 'tests/test_store_device.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' '-o' 'MDK/test_store_device.exe'
    if ($LASTEXITCODE -ne 0) { throw 'IAP register test compilation failed' }
    & '.\MDK\test_store_device.exe'
    if ($LASTEXITCODE -ne 0) { throw 'IAP register tests failed' }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' '-IUSER/inc' 'tests/test_pid.c' 'USER/src/track_pid.c' '-o' 'MDK/test_pid.exe'
    if ($LASTEXITCODE -ne 0) { throw 'PID test compilation failed' }
    & '.\MDK\test_pid.exe'
    if ($LASTEXITCODE -ne 0) { throw 'PID tests failed' }
    foreach ($count in @(2,4)) {
        & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' "-DTRACK_SENSOR_COUNT=$count" '-IUSER/inc' 'tests/test_core.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' 'USER/src/track_store.c' '-o' "MDK/test_core_$count.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Core test compilation failed' }
        & ".\MDK\test_core_$count.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Core tests failed' }
    }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' '-DTRACK_SENSOR_COUNT=4' '-IUSER/inc' 'tests/test_four.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' '-o' 'MDK/test_four.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Four-sensor reference test compilation failed' }
    & '.\MDK\test_four.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Four-sensor reference tests failed' }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-Itests/encoder_mocks' '-I../../Libraries/boards/inc' 'tests/test_encoder.c' '../../Libraries/boards/src/Encoder.c' '-o' 'MDK/test_encoder.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Encoder test compilation failed' }
    & '.\MDK\test_encoder.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Encoder tests failed' }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' '-IUSER/inc' 'tests/test_speed.c' 'USER/src/track_speed.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' '-o' 'MDK/test_speed.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Speed test compilation failed' }
    & '.\MDK\test_speed.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Speed tests failed' }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-Wno-format' '-DTRACK_HOST_TEST' '-IUSER/inc' '-Itests' '-I../../Libraries/boards/inc' 'tests/test_app.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' 'USER/src/track_speed.c' '../../Libraries/boards/src/Font.c' '-o' 'MDK/test_app.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Application test compilation failed' }
    & '.\MDK\test_app.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Application tests failed' }
    foreach ($servoHz in @(50,250,330)) {
        & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' "-DTRACK_SERVO_FREQUENCY=$servoHz" '-IUSER/inc' '-Itests' 'tests/test_hw.c' 'USER/src/track_hw.c' '-o' "MDK/test_hw_$servoHz.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Hardware layer test compilation failed' }
        & ".\MDK\test_hw_$servoHz.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Hardware layer tests failed' }
    }
    & $gcc '-std=c99' '-Wall' '-Wextra' '-Werror' '-DTRACK_HOST_TEST' '-DTRACK_PID_DEFAULT_KP=52.75f' '-DTRACK_PID_DEFAULT_KI=0.05f' '-DTRACK_PID_DEFAULT_KD=0.32f' '-IUSER/inc' 'tests/test_defaults.c' 'USER/src/track_core.c' 'USER/src/track_pid.c' 'USER/src/track_store.c' '-o' 'MDK/test_defaults.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Default/legacy compatibility test compilation failed' }
    & '.\MDK\test_defaults.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Default/legacy compatibility tests failed' }
} finally { Pop-Location }
