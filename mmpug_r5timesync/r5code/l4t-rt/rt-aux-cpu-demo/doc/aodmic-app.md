AODMIC Application (app/aodmic-app.c)
==============================
@tableofcontents

The AODMIC application demonstrates how to operate Always On (AON) DMIC
from the SPE/AON processor. AODMIC is in the AON domain for both
Jetson TX2 and Jetson AGX Xavier. The compilation of this demo app can be
controlled by the `ENABLE_AODMIC_APP` flag in the `soc/*/target_specific.mk`
file. The `ENABLE_GPCDMA_FUNC` flag must also be enabled.

The app runs continuously and captures data from AODMIC using GPCDMA. It
periodically prints the R5 CPU tick count and the zero crossing count (on both
capture channels). These counts can be used to determine whether the AODMIC is
running at the expected rate. The app also computes and prints the capture
volume over fixed periods (computed as the mean square of the samples). Should
the computed volume exceed a predefined threshold, it triggers system wake.

The command below may be executed on the running target in order to trigger
system suspend:
```
# sudo systemctl suspend
```

After the system is placed in the suspended state,
making a loud sound near the AODMIC mics should wake it up.

@note To demonstrate system wake, the corresponding wake event must be enabled
by CCPLEX/BPMP. In this case the wake event is wake83. When the system goes to
suspend state, the BPMP UART reports enabled masks of the wake events in the
log. Verify that in this log bit 83 is set, both in the wake mask and in the
Tier2 routing mask. For example:

    WAKE_MASK[95:64] = 0x1ff200
    TIER2[95:64] = 0xff200

@note The threshold for system wake may need to be modified so that it is
neither too high nor too low relative to the general volume of the
captured audio.

@note The AODMIC test app is currently configured for 16 kHz with stereo
channels. It may be modified to capture at any of the supported AODMIC rates.
OSR is fixed at 64.

# Jetson TX2 #

AODMIC signals are available at the 40-pin header J21 with these pin mappings:
- Pin 16 - DMIC5_DAT (CAN_GPIO0)
- Pin 32 - DMIC5_CLK (CAN_GPIO1)

@note Pinmux must be set for the J21 pins by following the instructions in
[*TX2 Developer Guide*](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fadaptation_and_bringup_tx2.html%23wwpID0E0BP0HA).

# Jetson AGX Xavier #

AODMIC signals are available at the 40-pin header J30 with these pin mappings:
- Pin 16 - DMIC5_DAT (CAN1_STB)
- Pin 32 - DMIC5_CLK (CAN1_EN)

@note Pinmux must be set for the J30 pins by following the instructions in
[*AGX Xavier Developer Guide*](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fadaptation_and_bringup_xavier.html%23wwpID0E0RN0HA).

@note If the system is placed in suspended state, BPMP is likely to disable
the dmic5 clock at the subsequent system wake, upon which AODMIC capture
will fail. You can patch this in either of two ways:
- On the running target execute:
```
# sudo su
# sudo echo 1 > sys/kernel/debug/bpmp/debug/clk/dmic5/state
```
*or*
- Patch the platform BPMP DT file (e.g. `tegra194-a02-bpmp-p2888-a04.dtb`)
  following these steps:

  - Reconstruct the device tree source file from the DTB:

        $ dtc -I dtb -O dts <BPMP DT file> -o <temp_file.dts>

  - Patch the third and fourth arguments to the 'dmic5' entry in the
    generated `temp_file.dts` file (third argument = <sample_rate*64>):

        clocks {
            lateinit {
                dmic5 = <0x86 0x5b 1024000 0x80000>;
            };
        };

  - Recreate the DTB file from the patched DTS:

        $ dtc -I dts -O dtb <temp_file.dts> -o <BPMP DT file>

  - Flash the `bpmp-fw-dtb` partition, or perform a full flash.
