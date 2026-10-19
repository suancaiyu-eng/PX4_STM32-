# Inject a software magnetometer-driver fault, not a physical field/line fault.
param()
$ErrorActionPreference='Stop'
$magProject=Split-Path -Parent $PSScriptRoot
$magMap=Get-Content -LiteralPath (Join-Path $magProject 'MDK-ARM\MPU6000\MPU6000.map') -Raw
$magMatch=[regex]::Match($magMap,'(?m)^\s+status\s+(0x[0-9a-fA-F]+)\s+Data\s+92\s+lsm303d_mag\.o')
if(!$magMatch.Success) { throw 'Cannot locate private magnetic-driver state' }
$magAddress=$magMatch.Groups[1].Value
$magCLI='D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe'
& $magCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u $magAddress 8 (Join-Path $PSScriptRoot 'fw8_mag_state_before_fault.bin') | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'fw8_mag_fault_injection.log')
if($LASTEXITCODE -ne 0) { throw 'Cannot read magnetic state' }
$magBefore=[System.IO.File]::ReadAllBytes((Join-Path $PSScriptRoot 'fw8_mag_state_before_fault.bin'))
if([BitConverter]::ToUInt32($magBefore,0)-ne 2 -or [BitConverter]::ToUInt32($magBefore,4)-ne 0) { throw 'Magnetometer was not healthy before injection' }
$magSerial=[System.IO.Ports.SerialPort]::new('COM5',460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$magSerial.Handshake=[System.IO.Ports.Handshake]::None
$magCapture=[System.Text.StringBuilder]::new()
try {
    $magSerial.Open(); $magSerial.DiscardInBuffer()
    & $magCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -w32 $magAddress 0x00000003 | Add-Content -LiteralPath (Join-Path $PSScriptRoot 'fw8_mag_fault_injection.log')
    if($LASTEXITCODE -ne 0) { throw 'Fault injection failed' }
    $magEnd=[DateTime]::UtcNow.AddSeconds(3)
    while([DateTime]::UtcNow -lt $magEnd) { [void]$magCapture.Append($magSerial.ReadExisting()); Start-Sleep -Milliseconds 5 }
} finally {
    if($magSerial.IsOpen) { $magSerial.Close() }
    $magSerial.Dispose()
    [System.IO.File]::WriteAllText((Join-Path $PSScriptRoot 'fw8_mag_fault_20261019.txt'),$magCapture.ToString())
}
& (Join-Path $PSScriptRoot 'read_fw8_state.ps1') -Tag fw8_mag_fault_state
Get-Content -LiteralPath (Join-Path $PSScriptRoot 'fw8_mag_fault_20261019.txt') -Tail 3
