# 达妙Orin 系列载板SDKmanager烧录方法

### 准备工作

- 硬件要求

|   |   |   |
|---|---|---|
|Host PC|![image](./images/Jetson_Orin_00.png)|Ubuntu 18.04/20.04(推荐)/22.04 x86_64，建议 50GB 以上 空闲硬盘，8GB以上 RAM|
|达妙Orin载板|![image](./images/Jetson_Orin_01.png)|Damiao_Orin_Board-V1/V2皆可(也可以是官方载板甚至其他第三方载板,但本文不做支持)|
|Jetson Orin Modules|![image](./images/Jetson_Orin_02.png)|Orin NX/Nano/-Super (4G、8G、16G)|
|USB数据线|![image](./images/Jetson_Orin_03.png)|USB Type-C 2.0/3.0(推荐)（用于 Host PC 与 Orin 连接）|
|固态硬盘|![image](./images/Jetson_Orin_04.png)|为安装系统镜像提供存储空间。Orin 模组的板载 eMMC 容量不足以支持镜像安装，需额外配备 SSD。建议容量大于 64GB 且健康状态良好|
|无线网卡|![image](./images/Jetson_Orin_05.png)|为设备提供网络连接。部分环境依赖网络支持，建议配备无线网卡或使用有线网络|
|主机网络|  |为主机设备提供网络连接。下载 SDK、Host 工具等组件依赖网络支持，建议配备无线网卡或使用有线网络|


- 下载烧录工具
    > Download link: https://developer.nvidia.com/sdk-manager

    ![image](./images/Jetson_Orin_06.png)

    - 安装软件
    ```bash
    sudo apt install ./sdkmanager_xx.xx.xxxx_amd64.deb
    ```

    - 打开软件
    ```bash
    sdkmanager
    ```


- 核心硬件安装
    - 安装并检查 Jetson_Orin 核心板是否安装好

    ![image](./images/Jetson_Orin_07.png)


    - 安装并检查固态硬盘SSD是否装好（必须检查），检查无线网卡是否装好（可选择安装）
    
    ![image](./images/Jetson_Orin_08.png)
    
- 连接设备

    *连接USB（input or OTG）在未上电状态按住REC键，随后打开电源开关（12V 3A），上电后继续按住REC按键，等待程序返回核心板详细信息即可松开。*

    ![image](./images/Jetson_Orin_09.png)


### 烧录
- 连接设备和选择固件版本
  - 链接设备，并选择模组

    ![image](./images/Jetson_Orin_10.png) 

  - 选择固件版本（6.0版本会下载Ubuntu22.04，5.1系列的会下载Ubuntu20.04）

    ![image](./images/Jetson_Orin_11.png) 

  - 烧录状态概览

    ![image](./images/Jetson_Orin_12.png) 
  

- 同意许可并选择烧录的文件
  - 完整功能下载

![image](./images/Jetson_Orin_13.png) 
  
  - 只下载Linux镜像

![image](./images/Jetson_Orin_14.png) 

- 输入本机密码（主机密码）继续操作

![image](./images/Jetson_Orin_15.png) 

- 设置系统用户名和密码

|推荐设置||
|---|---|
|Username|nvidia  (或更短的用户名)|
|Passwd  |nvidia  (或更短的密码)  |

![image](./images/Jetson_Orin_16.png) 

设置完成后点击Flash继续操作
- 下载和烧录固件

![image](./images/Jetson_Orin_17.png) 

- 安装cuda等开发组件

![image](./images/Jetson_Orin_18.png) 

- 烧录完成

![image](./images/Jetson_Orin_19.png) 


### 常见错误处理

| 错误类型 | 解决方法 |
|---------|---------|
| CUDA 组件安装失败 | [CUDA_component_install_failed.md](./CUDA_component_install_failed.md) |
| USB 连接失败 | [USB_connection_failed.md](./USB_connection_failed.md) |
| SSD 状态异常 | [SSD_status_failed.md](./SSD_status_failed.md) |