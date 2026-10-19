param([Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_]+$')][string]$Tag)
$ErrorActionPreference='Stop'
$magProject=Split-Path -Parent $PSScriptRoot
$magMap=Get-Content -LiteralPath (Join-Path $magProject 'MDK-ARM\MPU6000\MPU6000.map') -Raw
$magSymbols=@{}
foreach($magName in @('g_imu_status','g_telemetry_status','g_attitude','g_mag_status','g_mag_sample','g_spi_bus_status','g_attitude_max_update_us')) {
    $magMatch=[regex]::Match($magMap,'(?m)^\s+'+$magName+'\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s+')
    if(!$magMatch.Success) { throw ('Missing symbol: '+$magName) }
    $magSymbols[$magName]=@{address=[Convert]::ToUInt32($magMatch.Groups[1].Value.Substring(2),16);size=[int]$magMatch.Groups[2].Value}
}
$magBase=$magSymbols.g_imu_status.address
$magSize=$magSymbols.g_spi_bus_status.address+$magSymbols.g_spi_bus_status.size-$magBase
$magCLI='D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe'
& $magCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u ('0x{0:X8}' -f $magBase) $magSize (Join-Path $PSScriptRoot ($Tag+'.bin')) | Set-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'_read.log'))
if($LASTEXITCODE -ne 0) { throw 'Snapshot failed' }
& $magCLI -c port=SWD sn=35FF72063448303417211657 mode=HOTPLUG -u ('0x{0:X8}' -f $magSymbols.g_attitude_max_update_us.address) 4 (Join-Path $PSScriptRoot ($Tag+'_max.bin')) | Add-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'_read.log'))
if($LASTEXITCODE -ne 0) { throw 'Timing snapshot failed' }
@{base=$magBase;size=$magSize;symbols=$magSymbols;utc=[DateTime]::UtcNow.ToString('o')} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $PSScriptRoot ($Tag+'.json')) -Encoding utf8
Write-Output ('Saved FW8 snapshot: '+$Tag)
