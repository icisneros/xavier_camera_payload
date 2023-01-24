Compiling and Flashing {#rt-compiling}
======================

@tableofcontents

This Document describes the steps to build and flash this project

# TOOLCHAIN PREREQUISITE#

You must download the external Toolchain. NVIDIA does not distribute this toolchain.

https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update

# BUILD #

Set the appropriate paths for the below variables:
```
export TOP=<path to root directory where rt-aux-cpu-demo, hwinc and freertos related sources/directories reside>
export CROSS_COMPILE=<path to installed cross compiler>/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-
export FREERTOS_DIR=${TOP}/FreeRTOSV8.1.2/FreeRTOS/Source
export FREERTOS_COMMON_DIR=${TOP}/freertos-common

cd rt-aux-cpu-demo
```

## To build for T194 SoC based platforms ##

	make bin_t19x; To build only firmware binary

## To build for T186 SoC based platforms ##

	make bin_t18x; To build only firmware binary

## To build only doxygen documents ##

	make docs

## To build everything ##

	make
	or
	make all

The above commands build all the targets i.e. spe firmware binaries for all the
SOCs and Doxygen documents.

## Build artifacts ##

The built SPE firmware binary can be found at `$(OUTDIR)/<soc>/spe.bin`.
The Doxygen build should generate `$(OUTDIR)/docs/index.html` file. Open it
using browser to navigate through documents.

## To clean the build artifacts ##

	make clean; To clean firmware and doxygen build artifacts
	make clean_t18x; To clean t18x only firmware objects and build artifacts
	make clean_t19x; To clean t19x only firmware objects and build artifacts
	make clean_docs; To clean only doxygen generated files

# FLASH

1. Back up the original copies of `spe.bin` and `spe_t194.bin` located in the
following directory:

    Linux_for_Tegra/bootloader/

2. Copy the generated `${OUTDIR}/<soc>/spe.bin` to the following locating, depending on your target.
   - For the T186 SoC, copy it to:

         Linux_for_Tegra/bootloader/spe.bin
   - For the T194 SoC, copy it to:

         Linux_for_Tegra/bootloader/spe_t194.bin

   The `Linux_for_Tegra` directory is part of the extracted L4T build that
   you used to flash the Jetson device.
3. Use the command below to just flash this firmware for Cortex-R5 partition:

       sudo ./flash.sh -k spe-fw <T186 or T194 SoC based Jetson platforms> mmcblk0p1
