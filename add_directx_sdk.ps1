# Script to add DirectX SDK paths to vcxproj
$vcxprojPath = "Amalgam\Amalgam.vcxproj"
$dxsdkPath = "C:\Program Files (x86)\Microsoft DirectX SDK (June 2010)"
$dxsdkInclude = "$dxsdkPath\Include"
$dxsdkLibX86 = "$dxsdkPath\Lib\x86"
$dxsdkLibX64 = "$dxsdkPath\Lib\x64"

# Read the file
$content = Get-Content $vcxprojPath -Raw

# Add AdditionalIncludeDirectories to all Win32 ClCompile sections
$content = $content -replace '(<ClCompile>(?:(?!<\/ClCompile>).)*?<BufferSecurityCheck>false<\/BufferSecurityCheck>)', "`$1`r`n      <AdditionalIncludeDirectories>$dxsdkInclude;%(AdditionalIncludeDirectories)</AdditionalIncludeDirectories>"

# Add AdditionalLibraryDirectories and d3dx9.lib to all Win32 Link sections
$content = $content -replace '(<Link>.*?<AdditionalLibraryDirectories>\s*<\/AdditionalLibraryDirectories>)', "<Link>`r`n      <SubSystem>Console</SubSystem>`r`n      <GenerateDebugInformation>true</GenerateDebugInformation>`r`n      <AdditionalLibraryDirectories>$dxsdkLibX86;%(AdditionalLibraryDirectories)</AdditionalLibraryDirectories>"

# Add d3dx9.lib to AdditionalDependencies
$content = $content -replace '(<AdditionalDependencies>WindowsApp\.lib;)', '${1}d3dx9.lib;'

# Write back
Set-Content $vcxprojPath -Value $content -NoNewline

Write-Host "DirectX SDK paths added successfully!"
