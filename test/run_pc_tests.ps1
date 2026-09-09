# PC 测试桩一键跑测 (AGENTS.md §5.3 测试友好编码规范)
# 用法: .\test\run_pc_tests.ps1
$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)   # 切到仓库根

$gcc = "gcc"
if (-not (Get-Command $gcc -ErrorAction SilentlyContinue)) {
    $gcc = "D:\666Applications\All Tools\mingw64\bin\gcc.exe"
}

New-Item -ItemType Directory -Force -Path build | Out-Null

$incs = @("-I", "Core/Inc", "-I", "Core/Inc/driver", "-I", "Core/Inc/common")
$srcs = @("test/host_txt_cmd_test.c", "Core/Src/driver/txt_cmd.c")

Write-Host "== 编译 txt_cmd 测试桩 =="
& $gcc -Wall -Wextra @incs -o build/pc_test_txt_cmd.exe @srcs
if ($LASTEXITCODE -ne 0) { Write-Host "编译失败"; exit 1 }

& build\pc_test_txt_cmd.exe
exit $LASTEXITCODE
