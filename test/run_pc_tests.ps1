# PC 测试桩一键跑测 (AGENTS.md §5.3 测试友好编码规范 / 义务 7)
# 用法: .\test\run_pc_tests.ps1
$ErrorActionPreference = "Stop"
Set-Location (Split-Path $PSScriptRoot -Parent)   # 切到仓库根

$gcc = "gcc"
if (-not (Get-Command $gcc -ErrorAction SilentlyContinue)) {
    $gcc = "D:\666Applications\All Tools\mingw64\bin\gcc.exe"
}

New-Item -ItemType Directory -Force -Path build | Out-Null
$incs = @("-I", "Core/Inc", "-I", "Core/Inc/driver", "-I", "Core/Inc/common")

$failed = 0

Write-Host "== 编译+运行: txt_cmd 测试桩 =="
& $gcc -Wall -Wextra @incs -o build/pc_test_txt_cmd.exe `
    test/host_txt_cmd_test.c Core/Src/driver/txt_cmd.c
if ($LASTEXITCODE -ne 0) { Write-Host "编译失败"; exit 1 }
& build\pc_test_txt_cmd.exe
if ($LASTEXITCODE -ne 0) { $failed++ }

Write-Host ""
Write-Host "== 编译+运行: steering 执行组件测试桩 =="
& $gcc -Wall -Wextra @incs -o build/pc_test_steering.exe `
    test/host_steering_test.c Core/Src/driver/steering.c
if ($LASTEXITCODE -ne 0) { Write-Host "编译失败"; exit 1 }
& build\pc_test_steering.exe
if ($LASTEXITCODE -ne 0) { $failed++ }

Write-Host ""
if ($failed -eq 0) { Write-Host "== 全部测试桩通过 ==" } else { Write-Host "== 有 $failed 组测试失败 ==" }
exit $failed
