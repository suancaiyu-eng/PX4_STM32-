param([Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_]+$')][string]$Tag)
$ErrorActionPreference = 'Stop'
$attProject = Split-Path -Parent $PSScriptRoot
$attMap = Get-Content -LiteralPath (Join-Path $attProject 'MDK-ARM\MPU6000\MPU6000.map') -Raw
$attSymbols = @{}
foreach ($attName in @('g_imu_status','g_telemetry_status','g_attitude','g_attitude_max_update_us')) {
    $attMatch = [regex]::Match($attMap, '(?m)^\s+' + $attName + '\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s+')
    if (!$attMatch.Success) { throw ('Missing map symbol: ' + $attName) }
    $attSymbols[$attName] = @{ address=[Convert]::ToUInt32($attMatch.Groups[1].Value.Substring(2),16); size=[int]$attMatch.Groups[2].Value }
}
$attBase = $attSymbols.g_imu_status.address
$attSize = $attSymbols.g_attitude.address + $attSymbols.g_attitude.size - $attBase
$attCLI = 'D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe'
& $attCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u ('0x{0:X8}' -f $attBase) $attSize (Join-Path $PSScriptRoot ($Tag+'.bin')) | Set-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'_read.log'))
if ($LASTEXITCODE -ne 0) { throw 'Status upload failed' }
& $attCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u ('0x{0:X8}' -f $attSymbols.g_attitude_max_update_us.address) 4 (Join-Path $PSScriptRoot ($Tag+'_max.bin')) | Add-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'_read.log'))
if ($LASTEXITCODE -ne 0) { throw 'Timing upload failed' }
@{ base=$attBase; size=$attSize; symbols=$attSymbols; utc=[DateTime]::UtcNow.ToString('o') } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'.json')) -Encoding utf8
Write-Output ('Saved running-state snapshot: '+$Tag)
