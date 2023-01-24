export TOP=${PWD}/l4t-rt
export CROSS_COMPILE=${TOP}/../gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-
export OUTDIR=${TOP}/out
export FREERTOS_DIR=${TOP}/FreeRTOSV8.1.2/FreeRTOS/Source
export FREERTOS_COMMON_DIR=${TOP}/freertos-common
cd $TOP/rt-aux-cpu-demo
make TARGET=t19x
