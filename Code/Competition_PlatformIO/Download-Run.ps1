param([switch]$Check,[string]$RobotUrl='http://192.168.4.1')
$ErrorActionPreference='Stop'
$RobotUrl=$RobotUrl.TrimEnd('/')
function Get-RobotResponse([string]$Path) {
    for($attempt=1;$attempt -le 3;$attempt++) {
        try {return Invoke-WebRequest -UseBasicParsing -Uri "$RobotUrl$Path" -TimeoutSec 15 -Headers @{'Cache-Control'='no-cache'}}
        catch {if($attempt -eq 3){throw};Start-Sleep -Milliseconds 400}
    }
}
function Get-Hash([byte[]]$Data) {
    $hasher=[Security.Cryptography.SHA256]::Create()
    try {return ([BitConverter]::ToString($hasher.ComputeHash($Data))).Replace('-','').ToLowerInvariant()}
    finally {$hasher.Dispose()}
}
$suffix=if($Check){'?test=1'}else{''}
$manifest=(Get-RobotResponse "/export.json$suffix").Content | ConvertFrom-Json
# Both releases use the same CSV/export format. Already-installed 1.1 images
# advertise 1.0 in export.json because its label was accidentally hardcoded.
if($manifest.build -notin @('AshNazg competition 1.0','AshNazg competition 1.1','AshNazg competition 1.2')) {
    throw "Unsupported export format '$($manifest.build)'. Keep the robot powered on and stopped; send its summary."
}
if($manifest.capture_id -notmatch '^[a-fA-F0-9]+-[0-9]+$'){throw 'Invalid capture ID.'}
$folder=Join-Path $PSScriptRoot ('captures\'+$(if($Check){'check-'}else{'run-'})+$manifest.capture_id)
New-Item -ItemType Directory -Force -Path $folder | Out-Null
[IO.File]::WriteAllText((Join-Path $folder 'manifest.json'),($manifest | ConvertTo-Json),[Text.Encoding]::UTF8)
if(!$Check) {
    $summary=(Get-RobotResponse '/summary.txt').Content
    [IO.File]::WriteAllText((Join-Path $folder 'AshNazg-competition-summary.txt'),$summary,[Text.Encoding]::UTF8)
    if([int]$manifest.rows -eq 0){throw "No recorded rows. Summary saved in $folder; do not repeat or upload before inspecting it."}
}
$endpoint=if($Check){'/download-check.csv'}else{'/run.csv'}
$output=New-Object IO.MemoryStream
try {
    # Small pages avoid one long response. Each is independently checked before
    # saving, then the complete reconstructed CSV is checked against the manifest.
    for($start=0;$start -lt [int]$manifest.rows;$start+=25) {
        $response=Get-RobotResponse ($endpoint+'?start='+$start+'&count=25')
        [byte[]]$page=[Text.Encoding]::UTF8.GetBytes([string]$response.Content)
        $expectedHash=[string]$response.Headers['X-CSV-SHA256']
        if((Get-Hash $page) -ne $expectedHash){throw "Page checksum mismatch at row $start. Saved earlier pages in $folder."}
        [IO.File]::WriteAllBytes((Join-Path $folder ('part-'+$start.ToString('D4')+'.csv')),$page)
        $offset=0
        if($start -gt 0) {
            while($offset -lt $page.Length -and $page[$offset] -ne 10){$offset++}
            if($offset -eq $page.Length){throw 'CSV page has no header terminator.'};$offset++
        }
        $output.Write($page,$offset,$page.Length-$offset)
        Write-Progress -Activity 'Downloading robot recording' -Status "$start / $($manifest.rows) rows" -PercentComplete (100*$start/[int]$manifest.rows)
    }
    [byte[]]$all=$output.ToArray()
    if($all.Length -ne [int]$manifest.bytes -or (Get-Hash $all) -ne [string]$manifest.sha256){throw "Complete file verification failed; saved pages remain in $folder."}
    $finalName=if($Check){'AshNazg-download-check.csv'}else{'AshNazg-competition.csv'}
    [IO.File]::WriteAllBytes((Join-Path $folder $finalName),$all)
    Write-Progress -Activity 'Downloading robot recording' -Completed
    Write-Host "PASS: $($manifest.rows) rows, $($all.Length) bytes, SHA-256 verified."
    Write-Host "Saved in $folder"
    if($Check){Write-Host 'Download-only check passed. No preparation or motor commands were issued.'}
} finally {$output.Dispose()}
