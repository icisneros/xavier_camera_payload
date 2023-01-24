GPIO Application (app/gpio-app.c)
================================

@tableofcontents

GPIO application demonstrates how to access/manipulate Always On (AON) GPIOs from
the SPE/AON processor. Make sure that the kernel/bootloader is not accessing
these GPIOs (Usually, this can be done by modifying the device tree file) and no
external module is connected to the Jetson expansion header. Compilation of this
demo app can be controlled by `ENABLE_GPIO_APP` flag in the soc/*/
`target_specific.mk` file.

# Jetson TX2

In order to access a AON GPIO from the Cortex-R5 SPE/AON; the GPIO SCR and
Pinmux settings need to be updated to enable access to it as described in below
steps.

1. Update SCR values as below in the file mobile_scr.cfg:

        scr.1549.2 = 0x1C001010; # GPIO_Z_SCR_00_0
        scr.1550.2 = 0x1C001010; # GPIO_Z_SCR_01_0

2. Program the PADCTL register for the gpio pins as below in the file
   `tegra186-mb1-bct-pinmux-quill-p3310-1000-c03.cfg`:

         pinmux.0x0c303008 = 0x00000004; # can1_dout_pz0
         pinmux.0x0c303010 = 0x00000058; # can1_din_pz1

3. Flash the entire board to ensure that the SCR and pinmux settings are flashed
   on the board.

4. Short GPIO pins of the 30 pin header J26 pin 17 and pin 15, this should print
   out "GPIO input irq triggered" message since pin 17 is configured as OUT which
   drives the pin 15 which is configured as INPUT and also has interrupt enabled.

# Jetson AGX

In order to access a AON GPIO from the Cortex-R5 SPE/AON for Jetson AGX; the GPIO SCR,
GPIO interrupt map and pinmux settings need to be updated as described in below
steps.

1. Update SCR values as below in the file `tegra194-mb1-bct-scr-cbb-mini.cfg`:

         scr.41.6 = 0x18001010; # GPIO_BB_SCR_00_0
         scr.42.6 = 0x18001010; # GPIO_BB_SCR_01_0

2. Update gpio interrupt mapping as below in
   `tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg`:

         gpio-intmap.port.BB.pin.0 = 2; # GPIO BB0 to INT2
         gpio-intmap.port.BB.pin.1 = 2; # GPIO BB1 to INT2

3. Update default pinmux configuration as below in the file
   `tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`:

         pinmux.0x0c2f1800 = 0x00000003; # CONFIG BB0 output
         pinmux.0x0c303040 = 0x0000c004; # can1_stb_pbb0
         pinmux.0x0c303048 = 0x0000c044; # can1_en_pbb1

4. Compile device tree and flash the entire board to ensure that the SCR, gpio
   interrupt mapping and pinmux settings are flashed on the board.

5. Short GPIO pins of the 40 pin header J30 pin 16 and pin 32, this should print
   out "GPIO input irq triggered" message since pin 16 is configured as OUT which
   drives the pin 32 which is configured as INPUT and also has interrupt enabled.

# Jetson NX

In order to access a AON GPIO from the Cortex-R5 SPE/AON for Jetson NX,
`ENABLE_SPE_FOR_NX` flag has to be set to 1 in the `target_specific.mk` file
in the in the `soc/t19x`. The GPIO SCR, GPIO interrupt map and pinmux
settings need to be updated as described in below steps.

1. Update SCR values as below in the file `tegra194-mb1-bct-scr-cbb-mini-p3668.cfg`:

         scr.49.6 = 0x18001010; # GPIO_CC_SCR_04_0
         scr.53.6 = 0x18001010; # GPIO_DD_SCR_00_0

2. Update gpio interrupt mapping as below in
   `tegra194-mb1-bct-gpioint-p3668-0001-a00.cfg`:

         gpio-intmap.port.CC.pin.4 = 2; # GPIO CC4 to INT2
         gpio-intmap.port.DD.pin.0 = 2; # GPIO DD0 to INT2

3. Update default pinmux configuration as below in the file
   `tegra19x-mb1-pinmux-p3668-a01.cfg`:

         pinmux.0x0c302000 = 0x00000025; # touch_clk_pcc4: GPIO, pull-down, input-disable
         pinmux.0x0c302040 = 0x00000075; # gen2_i2c_sda_pdd0: i2c2, pull-down, input-enable

4. Compile device tree and flash the entire board to ensure that the SCR, gpio
   interrupt mapping and pinmux settings are flashed on the board.

5. Short GPIO pins of the 40 pin header J12 pin 15 and pin 27, this should print
   out "GPIO input irq triggered" message since pin 15 is configured as OUT which
   drives the pin 27 which is configured as INPUT and also has interrupt enabled.
