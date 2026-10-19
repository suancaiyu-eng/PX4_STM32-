$ErrorActionPreference = 'Stop'
$attCLI = 'D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe'
$attMaskFile = Join-Path $PSScriptRoot 'fw7_exti_before.bin'
& $attCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u 0x40013C00 4 $attMaskFile | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'fw7_fault_injection.log')
if ($LASTEXITCODE -ne 0) { throw 'Cannot read EXTI mask' }
$attMask = [BitConverter]::ToUInt32([System.IO.File]::ReadAllBytes($attMaskFile),0)
if ($attMask -ne 0x8000) { throw ('Unexpected EXTI mask, refusing fault injection: '+$attMask) }
$attSerial = [System.IO.Ports.SerialPort]::new('COM5',460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$attSerial.Handshake = [System.IO.Ports.Handshake]::None
$attCapture = [System.Text.StringBuilder]::new()
try {
    $attSerial.Open()
    $attSerial.DiscardInBuffer()
    & $attCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -w32 0x40013C00 0x00000000 | Add-Content -LiteralPath (Join-Path $PSScriptRoot 'fw7_fault_injection.log')
    if ($LASTEXITCODE -ne 0) { throw 'Cannot mask DRDY' }
    $attDeadline = [DateTime]::UtcNow.AddSeconds(4)
    while ([DateTime]::UtcNow -lt $attDeadline) {
        [void]$attCapture.Append($attSerial.ReadExisting())
        Start-Sleep -Milliseconds 5
    }
} finally {
    if ($attSerial.IsOpen) { $attSerial.Close() }
    $attSerial.Dispose()
    [System.IO.File]::WriteAllText((Join-Path $PSScriptRoot 'fw7_drdy_loss_20261019.txt'),$attCapture.ToString())
}
# Preserve latched-fault evidence before the caller explicitly resets the MCU.
& (Join-Path $PSScriptRoot 'read_fw7_state.ps1') -Tag fw7_fault_state
Get-Content -LiteralPath (Join-Path $PSScriptRoot 'fw7_drdy_loss_20261019.txt') -Tail 5
