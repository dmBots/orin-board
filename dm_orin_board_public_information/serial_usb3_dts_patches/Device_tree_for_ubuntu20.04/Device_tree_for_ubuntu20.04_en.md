# Device_tree_for_ubuntu20.04 <==>  适用于 Ubuntu 20.04 的设备树文件

> damiao_orin_borad is a third-party carrier board developed by Damiao Technology. So far, there are three versions: v1.0, v1.1, and v2.1. It is suitable for Nvidia Jetson Orin_NX / Orin_Nano series core modules.  
> During use, you may find that the USB2 (Type-C) interface on the carrier board is, by default, attached to the USB 2.0 bus, resulting in performance loss. In addition, you may also find that the UART 0 device on the carrier board cannot be used. Below, we will solve these problems together.

### Download the Nvidia Jetson Source Code

> Download link: https://developer.nvidia.com/embedded/jetson-linux-r3564

![device](../images/device_tree_20.png "Screenshot")

1. Download the BSP source code and the compilation toolchain separately

- Download `Driver Package (BSP) Sources` to obtain `public_sources.tbz2`

- Download `Bootlin Toolchain gcc 9.3` to obtain `aarch64--glibc--stable-final.tar.gz`

2. Build the compilation environment

```bash
mkdir -p jetson_linxu_35.6.4
cd jetson_linxu_35.6.4

# Add the files you downloaded into the current directory
# Be sure to replace `you_download_folder` below with your actual download directory. Do not copy it exactly as-is.
cp -r ~/you_download_folder/public_sources.tbz2 .
cp -r ~/you_download_folder/aarch64--glibc--stable-final.tar.gz .

# Extract the kernel files
tar -xjvf public_sources.tbz2

# Extract the compilation tools
mkdir -p aarch64--glibc--stable-final
tar -zxvf aarch64--glibc--stable-final.tar.gz -C aarch64--glibc--stable-final
```

![device](../images/device_tree_21.png "Screenshot")

2. Extract the source code part

```bash
cd you_kernel_ws/Linux_for_Tegra/source/public/

# You can extract the following three major components. They are all important, and none of them can be missing.
tar -xjvf kernel_src.tbz2
```

|   |   |
|---|---|
|`kernel_src.tbz2`|Linux kernel mainline source code|

### Add Build Tool Links and Environment Variables

1. Choose the corresponding compilation toolchain based on the operating platform

```bash
# Install the tool when operating on an amd (x86) host
sudo apt-get install device-tree-compiler

# Install the tool when operating on a Jetson board
sudo apt install tegra-21x-dt
sudo apt-get install libssl-dev
```

2. Add the compilation tool link in the current terminal

```bash
# Switch to the you_kernel_ws directory
cd ~/jetson_linxu_35.6.4/Linux_for_Tegra/source/public/kernel/kernel-5.10

# Be sure to replace `you_kernel_ws` below with your actual download directory. Do not copy it exactly as-is.
# First, set the JETPACK path
export JETPACK=~/jetson_linxu_35.6.4/Linux_for_Tegra

# Then set the output directory (using the JETPACK variable)
export KERNEL_OUT=$JETPACK/../images
export KERNEL_MODULES_OUT=$JETPACK/../images/modules

# Verify whether the path is correct
echo $KERNEL_OUT
# It should display: ~/jetson_linxu_35.6.4/Linux_for_Tegra/../images

# Create the output directories
mkdir -p $KERNEL_OUT
mkdir -p $KERNEL_MODULES_OUT

# Confirm the cross-compiler path
export CROSS_COMPILE=~/jetson_linxu_35.6.4/aarch64--glibc--stable-final/bin/aarch64-buildroot-linux-gnu-

# Verify the cross-compiler
${CROSS_COMPILE}gcc --version
```

*For example:*

![device](../images/device_tree_22.png "Screenshot")

### Modify the Device Tree

1. Modify the serial port section

**Modify the following part in `you_kernel_ws/Linux_for_Tegra/source/public/hardware/nvidia/platform/t23x/p3768/kernel-dts/cvb/tegra234-p3768-0000-a0.dtsi`**

- Add the `serial@3110000` section after the `serial@3100000` node and enable the serial port

```c
	mttcan@c310000 {
		status = "okay";
	};

	serial@3100000 {/* UARTA, for 40 pin header */
		status = "okay";
	};

	/* Add serial port UART1 corresponding to carrier board physical numbering UART0 */
	serial@3110000 {/* Enable UART1 */
		status = "okay";
	};

	serial@3140000 {
		/* UARTE, Goes to M2.E and also some of the pins to bootstrap */
		status = "okay";
	};
```

![device](../images/device_tree_23.png "Screenshot")

2. Modify the USB 3.0 section

- Under `xusb_padctl: xusb_padctl@3520000 --> pads --> usb3 --> lanes`, add the `usb3-2` section to add the third USB 3.0 channel configuration

```c
			usb3 {
				lanes {
					usb3-0 {
						nvidia,function = "xusb";
						status = "okay";
					};
					usb3-1 {
						nvidia,function = "xusb";
						status = "okay";
					};
					/* Add the lane configuration for the USB2 physical layer (PHY) */
					usb3-2 {	// The third USB 3.0 channel
						nvidia,function = "xusb";
						status = "okay";
					};
				};
			};
```

![device](../images/device_tree_24.png "Screenshot")

- Under `xusb_padctl: xusb_padctl@3520000 --> ports`, add the `usb3-2` section to activate the third USB 3.0 port

```c
		ports {
			usb2-0 {/* Goes to recovery port */
				mode = "otg";
				status = "okay";
				vbus-supply = <&p3768_vdd_5v_sys>;
				usb-role-switch;
				port {
					typec_p0: endpoint {
						remote-endpoint = <&fusb_p0>;
					};
				};
			};
			usb2-1 {/* Goes to hub */
				mode = "host";
				vbus-supply = <&p3768_vdd_av10_hub>;
				status = "okay";
			};
			usb2-2 {/* Goes to M2.E */
				mode = "host";
				vbus-supply = <&p3768_vdd_5v_sys>;
				status = "okay";
			};
			usb3-0 {/* Goes to hub */
				nvidia,usb2-companion = <1>;
				status = "okay";
			};
			usb3-1 {/* Goes to J5 */
				nvidia,usb2-companion = <0>;
				status = "okay";
			};
			usb3-2 {/* Add the usb3-2 section to enable USB 3.0 */
				nvidia,usb2-companion = <2>;		// "2" is the pairing identifier, meaning: channel usb3-2 is paired with channel usb2-2
				status = "okay";
			};
		};
```

![device](../images/device_tree_25.png "Screenshot")

- Connect the newly enabled hardware channel to the USB controller

```c
	tegra_xhci: xhci@3610000 {
		status = "okay";
		phys = <&{/xusb_padctl@3520000/pads/usb2/lanes/usb2-0}>,
			<&{/xusb_padctl@3520000/pads/usb2/lanes/usb2-1}>,
			<&{/xusb_padctl@3520000/pads/usb2/lanes/usb2-2}>,
			<&{/xusb_padctl@3520000/pads/usb3/lanes/usb3-0}>,
			<&{/xusb_padctl@3520000/pads/usb3/lanes/usb3-1}>,
			<&{/xusb_padctl@3520000/pads/usb3/lanes/usb3-2}>;		// Add this line to mount usb3-2 onto the USB 3.0 bus
		phy-names = "usb2-0", "usb2-1", "usb2-2", "usb3-0", "usb3-1", "usb3-2";		// Add usb3-2 and label usb3-2 on the bus for internal driver use
		nvidia,xusb-padctl = <&xusb_padctl>;
	};
```

![device](../images/device_tree_26.png "Screenshot")

### Compile the Kernel and Device Tree

1. Compile the kernel

```bash
# Enter Linux_for_Tegra/source/public/kernel/kernel-5.10
cd ~/you_kernel_ws/Linux_for_Tegra/source/public/kernel/kernel-5.10

# Set all environment variables
export JETPACK=~/jetson_linxu_35.6.4/Linux_for_Tegra
export KERNEL_OUT=$JETPACK/../images
export KERNEL_MODULES_OUT=$JETPACK/../images/modules
export CROSS_COMPILE=~/jetson_linxu_35.6.4/aarch64--glibc--stable-final/bin/aarch64-buildroot-linux-gnu-

# Configure the kernel
make ARCH=arm64 O=$KERNEL_OUT tegra_defconfig
```

![device](../images/device_tree_27.png "This means the kernel is being compiled")

2. Compile the device tree

```bash
# Set all environment variables
# export JETPACK=~/jetson_linxu_35.6.4/Linux_for_Tegra
# export KERNEL_OUT=$JETPACK/../images
# export KERNEL_MODULES_OUT=$JETPACK/../images/modules
# export CROSS_COMPILE=~/jetson_linxu_35.6.4/aarch64--glibc--stable-final/bin/aarch64-buildroot-linux-gnu-

# Compile the device tree
make ARCH=arm64 O=$KERNEL_OUT CROSS_COMPILE=$CROSS_COMPILE -j$(nproc) dtbs
```

![device](../images/device_tree_28.png "Press Enter to compile the device tree")

![device](../images/device_tree_29.png "The device tree compilation is now complete")

4. Check the generated device tree file

- The generated device tree file is usually located in `Linux_for_Tegra/../images/arch/arm64/boot/dts/nvidia`

```bash
cd ~/jetson_linxu_35.6.4/Linux_for_Tegra

ls ../images/arch/arm64/boot/dts/nvidia/
```

![device](../images/device_tree_30.png "Screenshot")

### Replace the Device Tree on the Board and Make the Bootloader Load It When Booting the Kernel

1. Replace the device tree for the Jetson Orin board

- Upload the generated `.dtb` file from the x86 host to the Jetson Orin board

```bash
scp -r ~/you_kernel_ws/tegra234-p3767-0004-p3768-0000-a0.dtb ssh ubuntu@example.com：~/
```

- On the Orin board, copy the obtained file into `/boot/dtb/`

```bash
sudo cp -r ~/tegra234-p3767-0004-p3768-0000-a0.dtb /boot/dtb/
```

2. Manually specify the path of the device tree file (`.dtb`) that the bootloader (U-Boot) should use when booting the kernel

- Add the following content into `/boot/extlinux/extlinux.conf` on the Jetson Orin board

```bash
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
	  # Add the line below to specify the corresponding dtb file.
      # Modify the filename according to your actual situation. You can run ls /boot/dtb to check.
      # FDT /boot/dtb/kernel_tegra234-p3767-0004-p3768-0000-a0.dtb
      FDT /boot/dtb/tegra234-p3767-0004-p3768-0000-a0.dtb
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=86c7e189-9e6e-41da-9d89-02ee429594d4 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 console=ttyAMA0,115200 firmware_class.path=/etc/firmware fbcon=map:0 net.ifnames=0 nv-auto-config

# When testing a custom kernel, it is recommended that you create a backup of
# the original kernel and add a new entry to this file so that the device can
# fallback to the original kernel. To do this:
#
# 1, Make a backup of the original kernel
#      sudo cp /boot/Image /boot/Image.backup
#
# 2, Copy your custom kernel into /boot/Image
#
# 3, Uncomment below menu setting lines for the original kernel
#
# 4, Reboot

# LABEL backup
#    MENU LABEL backup kernel
#    LINUX /boot/Image.backup
#    FDT /boot/dtb/kernel_tegra234-p3767-0004-p3768-0000-a0.dtb
#    INITRD /boot/initrd
#    APPEND ${cbootargs}
```

3. After completing the above steps, reboot the Jetson Orin board

### Test Whether USB and Serial Ports Are Enabled

- Insert a USB 3.0 device with a rate of (>=) USB 3.0, and check the output of `lsusb -t` in the command line

```bash
lsusb
lsusb -t
```

- Check whether all serial ports are listed

```bash
# Check whether /ttyTHS1 is listed (this is the serial port we enabled)
ls /dev/ttyTHS*
```

```log
nn@ubuntu:~$ lsusb
Bus 002 Device 009: ID 30de:6544 KIOXIA TransMemory     # A USB3 Gen2 removable storage device is detected
Bus 002 Device 007: ID 05e3:0625 Genesys Logic, Inc. USB3.2 Hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 005: ID 8087:0a2b Intel Corp. Bluetooth wireless interface
Bus 001 Device 015: ID 05e3:0610 Genesys Logic, Inc. Hub
Bus 001 Device 006: ID 046d:c52b Logitech, Inc. Unifying Receiver
Bus 001 Device 003: ID 1a40:0101 Terminus Technology Inc. Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
nn@ubuntu:~$ lsusb -t
/:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=tegra-xusb/4p, 10000M
    |__ Port 1: Dev 10, If 0, Class=Hub, Driver=hub/4p, 10000M
        |__ Port 4: Dev 11, If 0, Class=Mass Storage, Driver=usb-storage, 5000M     # Mounted onto the USB 3.0 bus at a speed of 500 Mbps
/:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=tegra-xusb/4p, 480M
    |__ Port 2: Dev 17, If 0, Class=Hub, Driver=hub/4p, 480M
    |__ Port 3: Dev 3, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 1: Dev 6, If 1, Class=Human Interface Device, Driver=usbhid, 12M
        |__ Port 1: Dev 6, If 2, Class=Human Interface Device, Driver=usbhid, 12M
        |__ Port 1: Dev 6, If 0, Class=Human Interface Device, Driver=usbhid, 12M
        |__ Port 4: Dev 5, If 0, Class=Wireless, Driver=btusb, 12M
        |__ Port 4: Dev 5, If 1, Class=Wireless, Driver=btusb, 12M
nn@ubuntu:~$
nn@ubuntu:~$
nn@ubuntu:~$ ls /dev/ttyTHS*
/dev/ttyTHS0  /dev/ttyTHS1  /dev/ttyTHS3  /dev/ttyTHS4		# Among them, /ttyTHS1 is the one we manually added and enabled
nn@ubuntu:~$
```

**When your device can be mounted on the USB 3.0 bus, and your serial ports all exist and are usable, it means our work has been successful!**

- At this point, your configuration work is basically complete. If you encounter any problems during use, you are welcome to submit an issue on Gitee. We will handle it as soon as possible. Please pay attention to the progress of your issue handling, and please do not submit duplicate issues (if someone has previously raised a similar problem).
- Thank you for using Damiao Technology products. We wish you a pleasant life and work!