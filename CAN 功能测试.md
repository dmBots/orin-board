# DM-Orin载板的CAN测试

**示例环境：**

**主机为 Ubuntu 20.04**

#### 测试工具

DM-Orin载板 ，达妙USBCAN分析仪

#### 测试步骤

1.将主机的CAN口和自制CAN收发模块的 CAN_H接CAN_H ，CAN_L接CAN_L

![图片.png](https://qiniu.md.amovlab.com/img/m/202301/20230113/1741161762164955441889280.png)

2.激活can

```
sudo modprobe mttcan
```

3.配置can波特率

```
sudo ip link set can0 type can bitrate 1000000
```

4.开启can

```
sudo ip link set up can0
```

5.接收数据：监视can0通道，在控制台上显示接收到的CAN帧。

```
sudo candump can0
```

![ORIN_CAN_SEND](./image/ORIN_CAN_SEND.png)

![ORIN_CAN](./image/ORIN_CAN.png)

6.发送CAN帧
发送CAN帧，可以使用下列命令,向can0通道发送一个4字节的CAN消息，并在CAN ID 001处设置11223344AABBCCDD的数据。
```
cansend can0 001#11223344AABBCCDD
```





















