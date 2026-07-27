@echo off
cd /d d:\666WorkSpace\STM32Cube\BluePill_Car
cmake --build build\Debug
if %errorlevel% neq 0 goto :end
STM32_Programmer_CLI.exe -c port=SWD -w build/Debug/BluePill_Car.elf 0x08000000 -rst
:end