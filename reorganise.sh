#!/bin/bash
set -e  # Exit immediately if any command fails

echo "==> Creating new folder structure..."
mkdir -p firmware hardware

echo "==> Moving STM32CubeIDE files into firmware/..."
mv .cproject .project .settings .mxproject firmware/
mv BuckBoostTest.ioc "BuckBoostTest Debug (1).launch" firmware/
mv STM32F446RETX_FLASH.ld STM32F446RETX_RAM.ld firmware/
mv Core Drivers firmware/

echo "==> Moving KiCad files into hardware/..."
mv SolarSystem.zip "Gerber Files.zip" "Footprints and Symbols.zip" hardware/
mv SolarSystem_BOM.csv hardware/

echo "==> Extracting KiCad zips..."
cd hardware
unzip -q SolarSystem.zip -d SolarSystem
unzip -q "Gerber Files.zip" -d gerbers
unzip -q "Footprints and Symbols.zip" -d libraries

echo "==> Removing original zip files..."
rm SolarSystem.zip "Gerber Files.zip" "Footprints and Symbols.zip"
cd ..

echo "==> Done! New structure:"
ls -la
echo ""
echo "firmware/ contents:"
ls -la firmware/
echo ""
echo "hardware/ contents:"
ls -la hardware/