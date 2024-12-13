@echo off
setlocal enabledelayedexpansion

:: Check if the correct number of arguments is passed
if "%~2"=="" (
    echo Usage: %0 [generation] [version]
    exit /b 1
)

:: Set variables for source and destination directories

:: XXX: Hardcoded source and destination directories
set "SOURCE_DIR=c:\Users\mTMS\mtms-fpga\FPGA Bitfiles"
set "DEST_DIR_INCLUDE=C:\Users\mTMS\mtms\ros2_ws\src\bridges\mtms_device_bridge\include"
set "DEST_DIR_LIB=C:\Users\mTMS\mtms\ros2_ws\src\bridges\mtms_device_bridge\src\lib"
set "DEST_DIR_BITFILES=C:\Users\mTMS\mtms\bitfiles"
set "GENERATION=%~1"
set "VERSION=%~2"

:: Copy files
echo Copying files...
echo %SOURCE_DIR%
echo %DEST_DIR_INCLUDE%
copy "%SOURCE_DIR%\NiFpga.h" "%DEST_DIR_INCLUDE%\NiFpga.h"
copy "%SOURCE_DIR%\NiFpga.c" "%DEST_DIR_LIB%\NiFpga.c"
copy "%SOURCE_DIR%\NiFpga_mTMS.h" "%DEST_DIR_INCLUDE%\NiFpga_mTMS.h"

:: Handle NiFpga_mTMS.lvbitx with dynamic naming
set "DEST_FILENAME=NiFpga_mTMS_generation_%GENERATION%_%VERSION%"
copy "%SOURCE_DIR%\NiFpga_mTMS.lvbitx" "%DEST_DIR_BITFILES%\%DEST_FILENAME%.lvbitx"

:: Extract hash from NiFpga_mTMS.h and create signature file
for /f "tokens=2 delims==" %%i in ('findstr "NiFpga_mTMS_Signature" "%DEST_DIR_INCLUDE%\NiFpga_mTMS.h"') do (
    set "HASH=%%i"
    set "HASH=!HASH:~2,-2!"  :: Remove quotes and trailing semicolon
    echo !HASH! > "%DEST_DIR_BITFILES%\%DEST_FILENAME%.signature"
)

echo Operation completed.

endlocal
