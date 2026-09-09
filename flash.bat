@echo off
cd /d d:\666WorkSpace\STM32Cube\BluePill_Car
rem 2026-09-09: Debug(-O0) 编译占 Flash 99.37%, 改用 Release(-Os) 仅 65.16%
cmake --build build\Release
if %errorlevel% neq 0 goto :end
STM32_Programmer_CLI.exe -c port=SWD -w build/Release/BluePill_Car.elf 0x08000000 -rst
::end
