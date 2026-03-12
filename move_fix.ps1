$file = 'src\main.cpp'
$lines = @(Get-Content $file -Encoding UTF8)

# Lines are 0-indexed, but we need to work with actual line numbers
# Find the firmware-panel start (should be around line 623, 0-indexed)
$fwStart = -1
$statusStart = -1
$scriptEnd = -1

for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match 'id="firmware-panel"') {
        $fwStart = $i
    }
    if ($lines[$i] -match 'class="status-grid"') {
        $statusStart = $i
    }
    if ($lines[$i] -match '</script>') {
        $scriptEnd = $i
    }
}

Write-Host "Found: firmware-panel at line $($fwStart+1), status-grid at line $($statusStart+1), </script> at line $($scriptEnd+1)"

if ($fwStart -ge 0 -and $statusStart -gt $fwStart -and $scriptEnd -gt $statusStart) {
    # Extract the panels (from firmware-panel to end of wifi-panel, which is before status-grid)
    $panelLines = $lines[$fwStart..($statusStart-1)]
    
    # Create new array:
    # 1. Lines before firmware panel
    # 2. Lines from status-grid to </script>
    # 3. The panels
    # 4. Lines after </script>
    
    $newLines = @()
    $newLines += $lines[0..($fwStart-1)]           # Before panels
    $newLines += $lines[$statusStart..$scriptEnd]  # Status grid and script
    $newLines += $panelLines                       # Panels moved here
    $newLines += $lines[($scriptEnd+1)..($lines.Count-1)]  # Footer and rest
    
    $newLines | Set-Content $file -Encoding UTF8 -NoNewline
    Write-Host "Successfully moved panels to end of page"
} else {
    Write-Host "Failed to find sections. fwStart=$fwStart, statusStart=$statusStart, scriptEnd=$scriptEnd"
}
