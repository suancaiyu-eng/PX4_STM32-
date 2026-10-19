param([string]$CapturePath, [int]$Seconds = 60)
$ErrorActionPreference = 'Stop'
$imuSerial = [System.IO.Ports.SerialPort]::new('COM5',460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$imuSerial.Handshake = [System.IO.Ports.Handshake]::None
$imuCapture = [System.Text.StringBuilder]::new()
$imuWatch = [System.Diagnostics.Stopwatch]::new()
try {
    $imuSerial.Open()
    $imuSerial.DiscardInBuffer()
    $imuWatch.Start()
    Write-Output ('Capturing COM5 for ' + $Seconds + ' seconds')
    while ($imuWatch.Elapsed.TotalSeconds -lt $Seconds) {
        $imuChunk = $imuSerial.ReadExisting()
        if ($imuChunk.Length -gt 0) { [void]$imuCapture.Append($imuChunk) }
        Start-Sleep -Milliseconds 5
    }
} finally {
    $imuWatch.Stop()
    if ($imuSerial.IsOpen) { $imuSerial.Close() }
    $imuSerial.Dispose()
    [System.IO.File]::WriteAllText($CapturePath,$imuCapture.ToString())
    @{ elapsed_seconds=$imuWatch.Elapsed.TotalSeconds; bytes=$imuCapture.Length } |
        ConvertTo-Json | Set-Content -LiteralPath ($CapturePath + '.json') -Encoding utf8
}
$imuLines = $imuCapture.ToString() -split "`n"
$imuLines | Select-Object -First 3
$imuLines | Select-Object -Last 3
Write-Output ('Bytes=' + $imuCapture.Length + ' Lines=' + $imuLines.Length + ' Seconds=' + $imuWatch.Elapsed.TotalSeconds)
