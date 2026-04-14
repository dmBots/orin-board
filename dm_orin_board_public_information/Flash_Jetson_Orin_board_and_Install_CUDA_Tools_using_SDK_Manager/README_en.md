# Damiao Orin Series Carrier Board SDK Manager Flashing Method

### Preparation

- Hardware Requirements

|   |   |   |
|---|---|---|
|Host PC|![image](./images/Jetson_Orin_00.png)|Ubuntu 18.04/20.04(Recommended)/22.04 x86_64, with at least 50GB of free disk space recommended, and 8GB or more RAM|
|Damiao Orin Carrier Board|![image](./images/Jetson_Orin_01.png)|Damiao_Orin_Board-V1/V2 supported (official carrier boards or other third-party carrier boards may also work, but are not covered in this document)|
|Jetson Orin Modules|![image](./images/Jetson_Orin_02.png)|Orin NX/Nano/-Super (4G, 8G, 16G)|
|USB Data Cable|![image](./images/Jetson_Orin_03.png)|USB Type-C 2.0/3.0 (3.0 recommended) (used for connecting the Host PC and Orin)|
|SSD|![image](./images/Jetson_Orin_04.png)|Provides storage space for installing the system image. The onboard eMMC capacity of the Orin module is insufficient to support image installation, so an additional SSD is required. A capacity greater than 64GB and good health status are recommended|
|Wireless Network Card|![image](./images/Jetson_Orin_05.png)|Provides network connectivity for the device. Some environments depend on network support, so it is recommended to equip a wireless network card or use wired networking|
|Host Network|![image](./images/Jetson_Orin_20.png)|Provides network connectivity for the host device. Downloading the SDK, Host tools, and other components depends on network access, so it is recommended to equip a wireless network card or use wired networking|


- Download the Flashing Tool
    > Download link: https://developer.nvidia.com/sdk-manager

    ![image](./images/Jetson_Orin_06.png)

    - Install the software
    ```bash
    sudo apt install ./sdkmanager_xx.xx.xxxx_amd64.deb
    ```

    - Launch the software
    ```bash
    sdkmanager
    ```


- Core Hardware Installation
    - Install and check whether the Jetson_Orin core module is installed properly

    ![image](./images/Jetson_Orin_07.png)


    - Install and check whether the SSD is installed properly (must be checked), and check whether the wireless network card is installed properly (optional)
    
    ![image](./images/Jetson_Orin_08.png)
    
- Connect the Device

    *Connect the USB cable (input or OTG) while the device is powered off, hold down the REC button, then turn on the power switch (12V 3A). After powering on, continue holding the REC button until the program returns detailed information about the core module, then release it.*

    ![image](./images/Jetson_Orin_09.png)


### Flashing
- Connect the device and select the firmware version
  - Connect the device and select the module

    ![image](./images/Jetson_Orin_10.png) 

  - Select the firmware version (version 6.0 will download Ubuntu 22.04, while the 5.1 series will download Ubuntu 20.04)

    ![image](./images/Jetson_Orin_11.png) 

  - Flashing status overview

    ![image](./images/Jetson_Orin_12.png) 
  

- Accept the license agreement and select the files to be flashed
  - Download the full feature package

![image](./images/Jetson_Orin_13.png) 
  
  - Download Linux image only

![image](./images/Jetson_Orin_14.png) 

- Enter the local machine password (host password) to continue

![image](./images/Jetson_Orin_15.png) 

- Set the system username and password

|Recommended Settings||
|---|---|
|Username|nvidia  (or a shorter username)|
|Passwd  |nvidia  (or a shorter password)  |

![image](./images/Jetson_Orin_16.png) 

After completing the setup, click Flash to continue.
- Download and flash the firmware

![image](./images/Jetson_Orin_17.png) 

- Install development components such as CUDA

![image](./images/Jetson_Orin_18.png) 

- Flashing completed

![image](./images/Jetson_Orin_19.png) 


### Common Error Handling

| Error Type | Solution |
|---------|---------|
| CUDA component installation failed | [CUDA_component_install_failed.md](./CUDA_component_install_failed.md) |
| USB connection failed | [USB_connection_failed.md](./USB_connection_failed.md) |
| SSD status abnormal | [SSD_status_failed.md](./SSD_status_failed.md) |
