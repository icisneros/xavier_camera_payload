UART application (app/uart-app.c)
=================================

@tableofcontents

UART application demonstrates how to access/manipulate Always On(AON) UARTs from
the SPE/AON processor. Make sure that the kernel/bootloader is not accessing
these UARTs (Usually, this can be done by modifying the device tree file).
Compilation of this demo app can be controlled by "ENABLE_UART_APP" flag
in the soc/*/target_specific.mk file.

# For Jetson TX2 #

There are 2 UART ports in AON cluster for Jetson TX2:
- UARTC (base address: 0x0c280000)
- UARTG (base address: 0x0c290000)

@note UARTG is also used by SPE firmware debug message output in Jetson TX2.
It's NOT recommended to use UARTG unless the end users know exactly how it
works.

In order to access AON UART from the Cortex-R5 SPE/AON; the UART SCR and
Pinmux settings need to be updated to enable access to it as described in below
steps.

1. Check UART SCR settings to make sure that SPE can access the module in the
file mobile_scr.cfg:

       scr.1862.2 = 0x38001414; # CLK_RST_CONTROLLER_AON_SCR_UARTC_0

2. Check the PADCTL register for the UART pins to make sure that they work in
   UART mode. `tegra186-mb1-bct-pinmux-quill-p3310-1000-a00.cfg`:

       pinmux.0x0c302020 = 0x00000400; # uart3_tx_pw2: uartc
       pinmux.0x0c302018 = 0x00000458; # uart3_rx_pw3: uartc

3. Flash the entire board to ensure that the SCR and pinmux settings are
flashed on the board.

4. Check kernel device tree to make sure that kernel will not initialize
UART module under test against the settings in R5 firmware.
For example, in R28.2.1, UARTC is configured in DTS as following by:
```
        serial@c280000 {
                compatible = "nvidia,tegra186-hsuart";
                status = "okay";
        };
```
Then kernel will re-initialize UARTC by high-speed mode, which will be
conflicted with R5 firmware sample code. Please comment out this part of
code and upgrade the kernel DTB.

After the UART test task runs, it will continuously output following messages
to desired UART port:
Message from SPE R5 UART
And also print received characters to debug port.

5. A simple way to test UARTC function in Jetson TX2 board.
5.1 Follow above steps from 1 to 4 and power up the device.
5.2 Use a wire to connect UARTC TX and Rx (in Jetson TX2 carrier board, they
are pin 5 and 4 in J17), then the following message will output from
SPE firmware debug UART port, i.e. UARTG:
Message from SPE R5 UART

# For Jetson AGX #

There are 2 UART ports in AON cluster for Jetson AGX:
- UARTC (base address: 0x0c280000) (See Warning below!)
- UARTG (base address: 0x0c290000)

@warning UARTC is the default system debug UART port in Jetson
AGX. PLEASE DO NOT ACCESS UARTC IN R5 FIRMWARE. OR THE SYSTEM MAY MALFUNCTION!

In order to access a AON UART from the Cortex-R5 SPE/AON for Jetson AGX; the
UART SCR, and pinmux settings need to be updated as described in below steps.

1. Check SCR values as below in the file tegra194-mb1-bct-scr-cbb-mini.cfg:

       scr.2609.6 = 0x18001616; # AON_NOC_UARTG_BLF_CONTROL_REGISTER_0

2. Check default pinmux configuration as below in the file
   `tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`.

       pinmux.0x0c302048 = 0x00000401; # spi2_sck_pcc0: uartg,
       pinmux.0x0c302050 = 0x00000459; # spi2_miso_pcc1: uartg,

3. Compile device tree and flash the entire board to ensure that the SCR,
   and pinmux settings are flashed on the board.

   After the UART test task runs, it will continuously output following messages
   to desired UART port:

       Message from SPE R5 UART
   It will also print received characters to debug port.

4. A simple way to test UARTG function in Jetson AGX board.

   4.1. Follow above steps from 1 to 3 and power up the device.

   4.2. Use a wire to connect UARTG TX and Rx (in Jetson AGX board, they are
      A5 and A6 in J6), then the following message will output from
      SPE firmware debug UART port:

          Message from SPE R5 UART
