$file = 'src\main.cpp'
$content = [IO.File]::ReadAllText($file, [Text.Encoding]::UTF8)

# Find the two panels and the status-grid that comes after them
$idx1 = $content.IndexOf("`t" + '<div class="control-panel" id="firmware-panel">')
$idx2 = $content.IndexOf("`t" + '<div class="status-grid">', $idx1)

if ($idx1 -gt 0 -and $idx2 -gt $idx1) {
    $panelsToMove = $content.Substring($idx1, $idx2 - $idx1)
    $beforePanels = $content.Substring(0, $idx1)
    $afterPanels = $content.Substring($idx2)
    
    # Find footer location
    $footerIdx = $afterPanels.IndexOf("`t" + '</script>')
    if ($footerIdx -gt 0) {
        $scriptEnd = "`t" + '</script>'
        $beforeFooter = $afterPanels.Substring(0, $footerIdx + $scriptEnd.Length)
        $footer = $afterPanels.Substring($footerIdx + $scriptEnd.Length)
        
        $newContent = $beforePanels + $beforeFooter + "`n`n" + $panelsToMove + "`n" + $footer
        
        [IO.File]::WriteAllText($file, $newContent, [Text.Encoding]::UTF8)
        Write-Host "Panels moved successfully to end of page"
    } else {
        Write-Host "Could not find footer"
    }
} else {
    Write-Host "Could not find panels. idx1=$idx1, idx2=$idx2"
}
