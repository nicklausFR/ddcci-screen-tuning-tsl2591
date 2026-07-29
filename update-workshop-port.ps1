param(
  [string]$Port = ""
)

$ErrorActionPreference = "Stop"

$sketchYaml = Join-Path $PSScriptRoot "sketch.yaml"

if (-not (Test-Path -LiteralPath $sketchYaml)) {
  throw "sketch.yaml not found in $PSScriptRoot"
}

if ([string]::IsNullOrWhiteSpace($Port)) {
  $previousErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  $boardList = & arduino-cli board list 2>&1 | ForEach-Object { $_.ToString() }
  $ErrorActionPreference = $previousErrorActionPreference
  $ports = @()

  foreach ($line in $boardList) {
    if ($line -match "^(COM\d+)\s+serial\s+") {
      $ports += $Matches[1]
    }
  }

  if ($ports.Count -eq 0) {
    Write-Host $boardList
    throw "No serial port detected. Connect the board, then run this script again."
  }

  if ($ports.Count -gt 1) {
    Write-Host "Multiple ports detected: $($ports -join ', '). Using $($ports[0])."
  }

  $Port = $ports[0]
}

if ($Port -notmatch "^COM\d+$") {
  throw "Invalid port: $Port. Expected format: COM8"
}

$content = Get-Content -LiteralPath $sketchYaml -Raw
$portPattern = "(?m)^(\s*port:\s*)\S+\s*$"

if ($content -notmatch $portPattern) {
  throw "Could not find a 'port: COMx' line in sketch.yaml"
}

$updated = $content -replace $portPattern, "`$1$Port"
Set-Content -LiteralPath $sketchYaml -Value $updated -NoNewline
Write-Host "Arduino Maker Workshop port updated to $Port in sketch.yaml"
