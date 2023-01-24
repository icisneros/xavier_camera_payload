Devicetree IVC
==============

The current IVC channels are described in the devicetree repository at:
- For Jetson TX2:

      tegra186-aon.dtsi
- For Jetson AGX:

      tegra194-aon.dtsi

The only channel supported in the current distribution is the echo channel.
The echo channel demonstrates:
- How to establish the IVC channel.
- How IVC communication works in the kernel.
- How IVC communication works in the Freertos code.


To add/remove or enable/disable the channels in linux kernel, you must update the
devicetree entries in the above dtsi file. The following
code shows how to enable the IVC echo channel:

	aon_echo {
		compatible = "nvidia,tegra186-aon-ivc-echo";
		mboxes = <&aon 0>;
		status = "okay";
	};

@note It is required to compile device tree and flash board to have device tree
updated after making kernel device tree changes as described above.

From the Linux kernel side, the
`/sys/devices/aon_echo/data_channel` node is used to communicate with AON
after enabling echo channel as mentioned above.

The echo channel ivc code in the current firmware resides at:

    rt-aux-cpu-demo/app/ivc-echo-task.c

And the channel descriptions are at:

    rt-aux-cpu-demo/platform/ivc-channel-ids.c


