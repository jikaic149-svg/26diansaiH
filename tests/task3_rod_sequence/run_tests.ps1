param(
    [string]$Compiler = "C:\msys64\ucrt64\bin\gcc.exe"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$includePath = Join-Path $repositoryRoot "APP\Gateway"
$sourcePath = Join-Path $includePath "task3_rod_sequence.c"
$testPath = Join-Path $PSScriptRoot "test_task3_rod_sequence.c"
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$compilerDirectory = Split-Path -Parent $compilerPath
$executablePath = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("task3_rod_sequence_test_{0}.exe" -f $PID)

$arguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Werror",
    "-fno-common",
    "-I$includePath",
    $sourcePath,
    $testPath,
    "-lm",
    "-o",
    $executablePath
)

try {
    $env:PATH = "$compilerDirectory;$env:PATH"
    & $compilerPath @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Host-test compilation failed with exit code $LASTEXITCODE"
    }
    & $executablePath
    if ($LASTEXITCODE -ne 0) {
        throw "Host tests failed with exit code $LASTEXITCODE"
    }
}
finally {
    Remove-Item -LiteralPath $executablePath -Force -ErrorAction SilentlyContinue
}
