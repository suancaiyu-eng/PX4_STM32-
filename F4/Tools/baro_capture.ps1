param(
    [Parameter(Mandatory=$true)][string]$CapturePath,
    [int]$Seconds=10,
    [string]$Port='COM5',
    [string]$ProgramHex,
    [switch]$Reset,
    [string]$ProbeSerial='35FF72063448303417211657'
)
$ErrorActionPreference='Stop'
$baroRoot=Split-Path -Parent $PSScriptRoot
$baroCLI='D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe'
$baroSerial=[System.IO.Ports.SerialPort]::new($Port,460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$baroSerial.Handshake=[System.IO.Ports.Handshake]::None
$baroSerial.DtrEnable=$false
$baroSerial.RtsEnable=$false
$baroText=[System.Text.StringBuilder]::new()
$baroWatch=[System.Diagnostics.Stopwatch]::new()
$baroOpenedUtc=$null
$baroCaptureStartedUtc=$null
try {
    $baroSerial.Open()
    $baroOpenedUtc=[DateTime]::UtcNow.ToString('o')
    $baroSerial.DiscardInBuffer()
    if($ProgramHex) {
        # Before replacing this board's firmware, retain the complete original.
        $baroBackup=Join-Path $baroRoot 'Logs/baro_before_flash.bin'
        if(!(Test-Path -LiteralPath $baroBackup) -or (Get-Item -LiteralPath $baroBackup).Length -ne 2097152) {
            throw 'A complete 2 MB original Flash backup is required before programming.'
        }
        & $baroCLI -c port=SWD "sn=$ProbeSerial" freq=1000 -w $ProgramHex -v -rst -run
        if($LASTEXITCODE -ne 0) { throw 'Programming/verification failed.' }
    } elseif($Reset) {
        & $baroCLI -c port=SWD "sn=$ProbeSerial" freq=1000 mode=HOTPLUG -rst -run
        if($LASTEXITCODE -ne 0) { throw 'Reset failed.' }
    }
    $baroWatch.Start()
    $baroCaptureStartedUtc=[DateTime]::UtcNow.ToString('o')
    while($baroWatch.Elapsed.TotalSeconds -lt $Seconds) {
        [void]$baroText.Append($baroSerial.ReadExisting())
        Start-Sleep -Milliseconds 5
    }
} finally {
    $baroWatch.Stop()
    if($baroSerial.IsOpen) { $baroSerial.Close() }
    $baroSerial.Dispose()
    [System.IO.File]::WriteAllText($CapturePath,$baroText.ToString())
    @{elapsed_seconds=$baroWatch.Elapsed.TotalSeconds;bytes=$baroText.Length;port=$Port;
      serial_open_utc=$baroOpenedUtc;capture_start_utc=$baroCaptureStartedUtc;
      capture_end_utc=[DateTime]::UtcNow.ToString('o');
      method='Continuous receive only; no synchronized movement events or height reference';
      accuracy_calibration=$false} |
        ConvertTo-Json | Set-Content -LiteralPath ($CapturePath+'.json') -Encoding utf8
}
$baroLines=$baroText.ToString() -split "`n"
$baroLines | Select-Object -First 4
$baroLines | Select-Object -Last 4
Write-Output ('Bytes='+$baroText.Length+' Lines='+$baroLines.Length)
