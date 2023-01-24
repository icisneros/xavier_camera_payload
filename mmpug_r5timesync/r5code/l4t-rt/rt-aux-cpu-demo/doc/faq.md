FAQ {#rt-faq}
===
@tableofcontents

This section answers frequently asked questions.

# Why do some modules stop working after a few seconds?

Some modules work after the device powers up but then stop working after a
few seconds. For example, I am configuring UART and GPIO in the application and
few seconds later application stops working.

@b Answer: This issue can happen when the kernel changes the configuration of a
module that SPE is using. This is because SPE runs before linux kernel. Possible
place to look is kernel device tree and making sure that module is disabled for
the kernel.

# Why do some modules fail to run or cause Cortex-R5 to crash/hang?

I have modules that fail to run even though I checked all codes and settings in
SPE.

@b Answer: This issue can occur for the following reasons:

- Incorrect SCR settings

  Some modules have Security Configuration Registers (SCR). Acessing a module
  that is configured with incorrect SCR settings may result in a system crash.

- Clocks are not enabled or properly not configured

# Why does my customized SPE firmware fail to flash?

I successfully flashed the default firmware. But then flashing failed with when I tried to flash
SPI firmware that I modified  to test my own code. I used the same flash command and the same
device.

@b Answer: Flashing customized SPE firmware can fail for several reasons:

- The custom SPE firmware has a problem, which causes the SPE firmware to crash and the device to
  hang. When that happens, the PC cannot communicate with the
  device and the flash command cannot proceed.

  To resolve this issue, carefully check the code you modified and ensure its logic and
  settings are correct.

- SPE firmware size exceeds limitation. To workaround this limitation, follow [Known Issues](rt-knownissues.html).
