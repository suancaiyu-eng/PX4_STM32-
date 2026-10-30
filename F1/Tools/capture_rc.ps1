param(
    [string]$Port='COM5',
    [ValidateRange(1,120)][int]$Seconds=5,
    [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Tag='rc_health',
    [ValidateSet('health','neutral','range')][string]$Phase='health',
    [switch]$WriteHeader
)
$ErrorActionPreference='Stop'
$rcProject=Split-Path -Parent $PSScriptRoot
$rcCapturePath=Join-Path $rcProject ('Logs\'+$Tag+'.txt')
$rcPort=[System.IO.Ports.SerialPort]::new($Port,460800,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$rcPort.Handshake=[System.IO.Ports.Handshake]::None
$rcPort.DtrEnable=$false
$rcPort.RtsEnable=$false
$rcText=[System.Text.StringBuilder]::new()
try {
    $rcPort.Open()
    $rcPort.DiscardInBuffer()
    $rcEnd=[DateTime]::UtcNow.AddSeconds($Seconds)
    $rcNext=[DateTime]::UtcNow
    while([DateTime]::UtcNow -lt $rcEnd) {
        [void]$rcText.Append($rcPort.ReadExisting())
        if([DateTime]::UtcNow -ge $rcNext) {
            [System.IO.File]::WriteAllText($rcCapturePath,$rcText.ToString())
            Write-Output ('Capture '+$Phase+': remaining '+[Math]::Ceiling(($rcEnd-[DateTime]::UtcNow).TotalSeconds)+' s; bytes='+$rcText.Length)
            $rcNext=[DateTime]::UtcNow.AddSeconds(5)
        }
        Start-Sleep -Milliseconds 10
    }
} finally {
    if($rcPort.IsOpen){$rcPort.Close()}
    $rcPort.Dispose()
    [System.IO.File]::WriteAllText($rcCapturePath,$rcText.ToString())
}
$rcArgs=@((Join-Path $PSScriptRoot 'r9ds_range.py'),'--input',$rcCapturePath,'--output',(Join-Path $rcProject ('Logs\'+$Tag+'.json')),'--phase',$Phase)
if($WriteHeader){$rcArgs+='--write-header'}
& 'C:\Users\21144\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' @rcArgs
if($LASTEXITCODE -ne 0){throw 'RC capture analysis failed; see capture and report.'}
