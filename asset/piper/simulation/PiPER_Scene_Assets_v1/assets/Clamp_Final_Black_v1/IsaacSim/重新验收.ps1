param(
  [string]$IsaacSimDir = $env:ISAAC_SIM_DIR,
  [ValidateSet('all','motion','drop','probe','clamp')][string]$Only = 'all',
  [int]$Rate = 0
)
if (-not $IsaacSimDir) { throw 'Set ISAAC_SIM_DIR or pass -IsaacSimDir.' }
$clampRoot = $PSScriptRoot
$clampPython = Join-Path $IsaacSimDir 'python.bat'
if (-not (Test-Path -LiteralPath $clampPython)) { throw '请通过 -IsaacSimDir 指定 Isaac Sim 安装目录。' }
$clampTests = if ($Only -eq 'all') { @('motion','drop','probe','clamp') } else { @($Only) }
foreach ($clampTest in $clampTests) {
  $clampRate = if ($Rate -gt 0) { $Rate } elseif ($clampTest -eq 'drop') { 960 } else { 480 }
  & $clampPython (Join-Path $clampRoot 'scripts/validate_runtime.py') --root $clampRoot --only $clampTest --rate $clampRate
  if ($LASTEXITCODE -ne 0) { throw 'Isaac Sim 运行错误，请检查日志。' }
  $clampResult = Get-Content -LiteralPath (Join-Path $clampRoot "evidence/${clampTest}_${clampRate}.json") -Raw | ConvertFrom-Json
  Write-Host "${clampTest} / ${clampRate} Hz：$($clampResult.status)"
  if ($clampResult.status -ne 'PASS') { exit 1 }
}
