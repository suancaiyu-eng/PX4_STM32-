param(
    [string]$CapturePath,
    [int]$Seconds = 8,
    [switch]$ResetOnly
)
$ErrorActionPreference = 'Stop'
$imuSerial = [System.IO.Ports.SerialPort]::new('COM5',460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$imuSerial.Handshake = [System.IO.Ports.Handshake]::None
$imuSerial.DtrEnable = $false
$imuSerial.RtsEnable = $false
$imuCapture = [System.Text.StringBuilder]::new()
try {
    $imuSerial.Open()
    $imuSerial.DiscardInBuffer()
    if ($ResetOnly) {
        & 'D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe' -c port=SWD sn=35FF72063448303417211657 freq=1000 mode=HOTPLUG -rst -run
    } else {
        & 'D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe' -c port=SWD sn=35FF72063448303417211657 freq=1000 -w 'D:\pixhawk2.4.8\code_keil\MPU6000\MDK-ARM\MPU6000\MPU6000.hex' -v -rst -run
    }
    if ($LASTEXITCODE -ne 0) { throw 'Programming failed' }
    $imuCaptureUntil = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $imuCaptureUntil) {
        $imuChunk = $imuSerial.ReadExisting()
        if ($imuChunk.Length -gt 0) { [void]$imuCapture.Append($imuChunk) }
        Start-Sleep -Milliseconds 10
    }
} finally {
    if ($imuSerial.IsOpen) { $imuSerial.Close() }
    $imuSerial.Dispose()
    [System.IO.File]::WriteAllText($CapturePath,$imuCapture.ToString())
}
$imuLines = $imuCapture.ToString() -split "`n"
$imuLines | Select-Object -First 16
$imuLines | Select-Object -Last 4
Write-Output ('Bytes=' + $imuCapture.Length + ' Lines=' + $imuLines.Length)
