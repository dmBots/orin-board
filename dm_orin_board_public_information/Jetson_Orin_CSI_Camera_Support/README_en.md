# CSI Camera Adaptation and Usage Guide for DaMiao Orin Series Carrier Boards

> The current official Jetson driver only natively supports the IMX219 sensor. It is recommended to prioritize this model. In general, IMX219 camera modules with a 15-pin FPC connector should work properly.

### Pin Details

- CSI0

| Connector Pin | Signal | Module Pin | Type |
|---|---|---|---|
| 1  | GND         | --  | Power |
| 2  | CSI1_D0_N   | 3   | CIS Differential |
| 3  | CSI1_D0_P   | 5   | CIS Differential |
| 4  | GND         | --  | Power |
| 5  | CSI1_D1_N   | 15  | CIS Differential |
| 6  | CSI1_D1_P   | 17  | CIS Differential |
| 7  | GND         | --  | Power |
| 8  | CSI1_CLK_N  | 9   | CIS Differential |
| 9  | CSI1_CLK_P  | 11  | CIS Differential |
| 10 | GND         | --  | Power |
| 11 | CSI0_D0_N   | 4   | CIS Differential |
| 12 | CSI0_D0_P   | 6   | CIS Differential |
| 13 | GND         | --  | Power |
| 14 | CSI0_D1_N   | 16  | CIS Differential |
| 15 | CSI0_D1_P   | 18  | CIS Differential |
| 16 | GND         | --  | Power |
| 17 | CAM0_PWDN_LS| 114 | CMOS 1.8V Output |
| 18 | CAM0_MCLK   | 116 | CMOS 1.8V Output |
| 19 | GND         | --  | Power |
| 20 | CAM0_I2C_SCL| *   | Open-Drain 3.3V |
| 21 | CAM0_I2C_SDA| *   | Open-Drain 3.3V |
| 22 | 3.3V        | --  | Power |

*Note: This part of the circuit is identical to the original design. IIC is routed from CAM_I2C_SDA (215) and CAM_I2C_SCL (213) through CAM_MUX_SEL (130).*

![image](./images/camera_CSI_00.png)

- CSI1

| Connector Pin | Signal | Module Pin | Type |
|---|---|---|---|
| 1  | GND          | --  | Power |
| 2  | CSI2_D0_N    | 22  | CIS Differential |
| 3  | CSI2_D0_P    | 24  | CIS Differential |
| 4  | GND          | --  | Power |
| 5  | CSI2_D1_N    | 34  | CIS Differential |
| 6  | CSI2_D1_P    | 36  | CIS Differential |
| 7  | GND          | --  | Power |
| 8  | CSI2_CLK_N   | 28  | CIS Differential |
| 9  | CSI2_CLK_P   | 30  | CIS Differential |
| 10 | GND          | --  | Power |
| 11 | CSI3_D0_N    | 21  | CIS Differential |
| 12 | CSI3_D0_P    | 23  | CIS Differential |
| 13 | GND          | --  | Power |
| 14 | CSI3_D1_N    | 33  | CIS Differential |
| 15 | CSI3_D1_P    | 35  | CIS Differential |
| 16 | GND          | --  | Power |
| 17 | CAM1_PWDN_LS | 120 | CMOS 1.8V Output |
| 18 | CAM1_MCLK    | 122 | CMOS 1.8V Output |
| 19 | GND          | --  | Power |
| 20 | CAM1_I2C_SCL | *   | Open-Drain 3.3V |
| 21 | CAM1_I2C_SDA | *   | Open-Drain 3.3V |
| 22 | 3.3V         | --  | Power |

*Note: This part of the circuit is identical to the original design. IIC is routed from CAM_I2C_SDA (215) and CAM_I2C_SCL (213) through CAM_MUX_SEL (130).*

![image](./images/camera_CSI_01.png)

- IMX29 Pin Assignment

| No. | Name   | Pin Type | Description |
|---|---|---|---|
| 1  | GND     | Ground |  |
| 2  | MDN0    | O      | MIPI data positive output |
| 3  | MDP0    | O      | MIPI data negative output |
| 4  | GND     | Ground |  |
| 5  | MDN1    | O      | MIPI data positive output |
| 6  | MDP1    | O      | MIPI data negative output |
| 7  | GND     | Ground |  |
| 8  | MCN     | O      | MIPI clock negative output |
| 9  | MCP     | O      | MIPI clock positive output |
| 10 | GND     | Ground |  |
| 11 | RESET   | I      | Reset |
| 12 | FSTROBE | O      | Strobe output |
| 13 | SCL     | I      |  |
| 14 | SDA     | I/O    |  |
| 15 | VCC3.3V | Power  |  |

*The figure below shows the interface definition of a standard 15-pin CSI module. This figure is taken from the IMX219 module specification (source: [SparkFun](https://cdn.sparkfun.com/assets/0/b/0/e/d/LI-IMX219-MIPI-FF-NANO_SPEC.pdf)). The camera module used in this solution has exactly the same interface definition, so you can refer directly to this figure for hardware connection.*

![image](./images/camera_CSI_02.png "Image source: https://cdn.sparkfun.com/assets/0/b/0/e/d/LI-IMX219-MIPI-FF-NANO_SPEC.pdf")
<!-- [![image](./images/camera_CSI_02.png)](https://cdn.sparkfun.com/assets/0/b/0/e/d/LI-IMX219-MIPI-FF-NANO_SPEC.pdf) -->
<!-- ![15-pin CSI interface diagram](./images/camera_CSI_02.png) -->

### Connecting an IMX219 Camera to the Orin Carrier Board

![image](./images/camera_CSI_03.png)

![image](./images/camera_CSI_04.png)

- Prepare a 15-pin to 22-pin FPC cable (as shown below) to connect the camera module to the Orin carrier board.

![image](./images/camera_CSI_05.png)

- Be sure to verify the VCC and GND pin order of the cable

    - On the DaMiao Orin carrier board CSI connector, pin 1 is GND

    - On the IMX219 camera module, pin 1 is also GND

- The CSI connector on the DaMiao Orin carrier board supports both cable orientations. You can adjust the connection method based on the actual cable pin order. However, make sure it is not connected in reverse. Otherwise, the camera module, carrier board, or even the core board may be burned out and permanently damaged beyond repair.

### Configure the Camera

- Check whether the camera is present

```bash
# Check whether the camera is recognized by the system
sudo dmesg | grep imx
```

- Confirm the I2C address and scan

```bash
# List all I2C buses
ls /dev/i2c-*

# Scan a specific bus, for example i2c-9
sudo i2cdetect -r -y 9
```

- Check which camera devices are recognized by the system

```bash
ls -la /dev/video*
v4l2-ctl --list-devices
```

- Install the required tools

```bash
# Install v4l-utils (includes the v4l2-ctl command)
sudo apt-get update
sudo apt-get install v4l-utils

# Also install GStreamer tools (for preview and recording)
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

- Check the formats and resolutions supported by the camera

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

![image](./images/camera_CSI_06.png)

### Capture Video with nvgstcapture-1.0

- Basic preview and photo capture

```bash
# Open the preview window (press J to take a photo, press Q to quit)
nvgstcapture-1.0
```

## Common Shortcut Keys for nvgstcapture-1.0

```bash
# Open the preview window (press J to take a photo, press Q to quit)
nvgstcapture-1.0
```

![image](./images/camera_CSI_07.png)

| Key | Function |
|------|------|
| J | Take and save a photo |
| Q | Quit the program |
| Space | Start/stop recording |
| + / - | Zoom |
| ] / [ | Adjust brightness |

![image](./images/camera_CSI_08.png)

***The figure above shows the real-time preview screen after nvgstcapture-1.0 runs successfully. If you can see a clear moving image, it means: the IMX219 camera hardware connection is correct; the driver has been loaded successfully; and the image capture pipeline is working properly.***

### Closing Remarks

- At this point, your IMX219 camera configuration is basically complete. By following the steps above, you should be able to successfully detect the camera and capture real-time video.

- If you encounter any issues during use, you are welcome to submit an issue on Gitee. We will handle it as soon as possible. Please keep an eye on the progress of your issue.

    - Please first search for similar issues on Gitee to avoid duplicate submissions

    - If no solution is found, you are welcome to submit a new issue on Gitee, and we will handle it as soon as possible

    - Please keep track of your issue progress so that follow-up feedback can be provided in a timely manner

- Thank you for using DaMiao Technology products. We wish you a pleasant life and work.