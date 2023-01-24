# Time synchronization with Jetson Sensor Processing Engine (SPE)
This repository is a package for time synchronization using Cortex-R5 on Jetson 
Xavier.

## Device tree and configuration settings
### IVC data channel
This device tree change enables a data channel between CCPLEX (Linux) and SPE.

1. Download public source for L4T Xavier.
2. Uncompress Kernel folder.
3. In `hardware/soc/t19x/kernel-dts/tegra194-soc/tegra194-aon.dtsi`, add the 
following:  

        aon_echo {
            compatible = "nvidia,tegra186-aon-ivc-echo";
            mboxes = <&aon 0>;
            status = "okay";
        };

4. Build the device tree. Refer to [this documentation](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fkernel_custom.html%23wwpID0E0ZC0HA).  
    You will need to download a cross compiler toolchain for this.
    To only build the device tree, run:

        make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig
        make ARCH=arm64 O=$TEGRA_KERNEL_OUT dtbs

    Copy the compiled device tree under `TEGRA_KERNEL_OUT/arch/arm64/boot/dts/` 
    to `Linux_for_Tegra/kernel/dtb/`
    You should see `/sys/devices/aon_echo/data_channel` node after reflash. 
Refer to the `Flash the Xavier` section for instructions on how to flash the 
Xavier.

### GPIO settings
The package uses pin 16 and pin 32 for triggering.
In `Linux_for_Tegra/bootloader/t186ref/BCT` folder:

1. Change `-bct-scr-cbb-mini.cfg`:  

        scr.41.6 = 0x18001010; # GPIO_BB_SCR_00_0
        scr.42.6 = 0x18001010; # GPIO_BB_SCR_01_0
        scr.1867.4 = 0x3900ffff; # TKE_TOP_SCR_TKESCR_0 (Allows use of TMR_SHARED for TMRATR)

2. Change `tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg`:  

        gpio-intmap.port.BB.pin.0 = 2; # GPIO BB0 to INT2
        gpio-intmap.port.BB.pin.1 = 2; # GPIO BB1 to INT2

3. Change `tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`:  

        pinmux.0x0c2f1800 = 0x00000003; # CONFIG BB0 output
        pinmux.0x0c303040 = 0x0000c004; # can1_stb_pbb0
        pinmux.0x0c303048 = 0x0000c044; # can1_en_pbb1

_**Note: Some of the above configurations have multiple entries. Make sure to 
modify them all.**_

## Building the firmware
We need to use gcc cross compiler to compile the SPE firmware, and flash it 
onto the Xavier.

1. Download [GNU Arm Embedded Toolchain](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update) 
for cross compile.
2. Extract and place the toolchain folder (`gcc-arm-none-eabi-4_8-2014q3`) to 
the folder `r5code`.
3. Build the firmware with `build.sh`. Change permission if necessary.
4. Copy the output binary to L4T folder with following command: 

        cp l4t-rt/out/spe.bin XXX/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P2888/Linux_for_Tegra/bootloader/spe_t194.bin 

    Change the path accordingly if you are using a different version of Jetpack.

## Flash the entire Xavier
Now that we have the modified device tree, modified pinmux configuration, and 
SPE firmware in `Linux_for_Tegra` directory, we can flash the entire Xavier.

1. Redirect to the 
`nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P2888/Linux_for_Tegra` directory.
1. Put Xavier into forced recovery mode. By holding the middle button, press 
power button, and release both.
1. Check if Xavier is shown up as a USB device. There should be a device with 
name `NVida Corp.`.
1. Flash the Xavier with command `sudo flash.sh jetson-xavier mmcblk0p1`.

_**Note1: Flash the entire Xavier will erase everything. Backup if necessary.**_  
_**Note2: You don't need to flash the entire Xavier if you only changed the 
SPE firmware. Flash with option `-k spe-fw` to only flash SPE firmware.**_


