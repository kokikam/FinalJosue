# Define the path to your CSV and the folder to check
$csvPath = "C:\Users\Josue\OneDrive\MNA\Navegacion Autonoma\FinalJosue\data.csv"
$outputPath = "C:\Users\Josue\OneDrive\MNA\Navegacion Autonoma\FinalJosue\filtered.csv"
$folderPath = "C:\Users\Josue\OneDrive\MNA\Navegacion Autonoma\FinalJosue\data"

# Import CSV
$data = Import-Csv -Path $csvPath

# Filter records where file exists
$filtered = $data | Where-Object {
    $fileName = $_.Column1
    Test-Path -Path (Join-Path $folderPath $fileName)
}

# Export the filtered data to a new CSV
$filtered | Export-Csv -Path $outputPath -NoTypeInformation

Write-Host "Filtered CSV saved to $outputPath"