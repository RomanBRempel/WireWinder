$file = 'src\main.cpp'

# Read all lines
$content = [System.IO.File]::ReadAllLines($file, [System.Text.Encoding]::UTF8)
Write-Host "Total lines: $($content.Count)"

# Find indices by scanning for the HTML elements
$fwStart = -1
$wifiStart = -1
$statusStart = -1
$scriptEnd = -1

for ($i = 0; $i -lt $content.Count; $i++) {
    if ($fwStart -lt 0 -and $content[$i].Contains('<div class="control-panel" id="firmware-panel">')) {
        $fwStart = $i
    }
    if ($wifiStart -lt 0 -and $content[$i].Contains('<div class="control-panel" id="wifi-panel">')) {
        $wifiStart = $i
    }
    if ($statusStart -lt 0 -and $content[$i].Contains('<div class="status-grid">')) {
        $statusStart = $i
    }
    if ($content[$i].Contains('</script>')) {
        $scriptEnd = $i
    }
}

Write-Host "fw: $fwStart, wifi: $wifiStart, status: $statusStart, script: $scriptEnd"

if ($fwStart -ge 0 -and $statusStart -gt $wifiStart -and $scriptEnd -gt $statusStart) {
    # Build new lines array
    $newLines = @()
    
    # Part 1: Lines before firmware panel (0 to fwStart-1)
    $newLines += $content[0..($fwStart-1)]
    
    # Part 2: Status grid through </script> (statusStart to scriptEnd)
    $newLines += $content[$statusStart..$scriptEnd]
    
    # Part 3: Firmware panel through end of wifi panel (fwStart to statusStart-1)
    $newLines += $content[$fwStart..($statusStart-1)]
    
    # Part 4: Everything after </script>
    if ($scriptEnd -lt $content.Count - 1) {
        $newLines += $content[($scriptEnd+1)..($content.Count-1)]
    }
    
    [System.IO.File]::WriteAllLines($file, $newLines, [System.Text.Encoding]::UTF8)
    Write-Host "Successfully reorganized! Moved panels from position $fwStart-$($statusStart-1) to after line $scriptEnd"
} else {
    Write-Host "ERROR: Could not find all sections"
}
