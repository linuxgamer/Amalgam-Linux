$vcxproj = "Amalgam/Amalgam.vcxproj"
$content = Get-Content $vcxproj -Raw

# Add WindowsApp.lib after AdditionalLibraryDirectories in all Link sections
$pattern = '(<AdditionalLibraryDirectories>\s*</AdditionalLibraryDirectories>)'
$replacement = '$1`n      <AdditionalDependencies>WindowsApp.lib;%(AdditionalDependencies)</AdditionalDependencies>'

$newContent = $content -replace $pattern, $replacement

$newContent | Set-Content $vcxproj -NoNewline

Write-Host "WindowsApp.lib added to all configurations"
