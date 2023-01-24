SPI application (app/spi-app.c)
==============================

@tableofcontents

SPI application demonstrates how to access/manipulate Always On (AON) SPI from
the SPE/AON processor. SPI 2 is in AON domain for both
Jetson TX2 and Jetson AGX. The
compilation of this demo app can be controlled by "ENABLE_SPI_APP" flag in the
soc/*/target_specific.mk file. The app uses loop back test, user must short
MISO and MOSI signals (follow below sections for pin map) in order to
successfully run this app. The app sends predefined bytes and compares sent
bytes with received, on success it should print "SPI test successful".

@note It is required to compile device tree and flash board to have device tree
updated after making kernel device tree changes as described in following
sections.

# Jetson TX2 #

SPI 2 signals are available at connector J23 with below pin mappings.

- Pin 38 - CLK
- Pin 40 - MISO
- Pin 42 - MOSI
- Pin 44 - CS0

SPI 2 is also enabled and accessed in tegra186-display-e3320-1000-a00.dtsi
device tree file from kernel side which needs to be disabled as shown in below
code snippet.

```
spi@c260000 {
        status = "disabled";
        spi-max-frequency = <12000000>;
        ...
}
```

# Jetson AGX #

SPI 2 signals are available at connector J6 with below pin mappings.

- Pin A5 - CLK
- Pin A6 - MISO
- Pin A7 - MOSI
- Pin A8 - CS0

SPI 2 is also enabled and accessed in tegra194-camera-e3377-a00.dtsi
device tree file from kernel side which needs to be disabled as shown in below
code snippet.

```
spi@c260000 {
        status = "disabled";
        spi-max-frequency = <12000000>;
        ...
}
```
