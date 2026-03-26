# DM_Orin_board 开发板资料目录

### 总览
```bash
orin-board$ tree -L 2
.
├── CAN功能测试.md
├── dm_orin_board_public_information		# 通用教程目录，适用于v1、v2载板
│   ├── iwlwifi-offline						# Intel 无线网卡驱动安装
│   ├── rtc设置.rar 
│   ├── serial_usb3_dts_patches				# 设备树适配
│   └── 使能UART方法
├── dm_orin_board_v1_information		# DM_Orin_Board_V1系列资料
│   ├── 2D标注
│   ├── Orin_NX-RJ45-V1.0.pdf
│   └── 说明书		# v1版本说明书
├── dm_orin_board_v2_information		# DM_Orin_Board_V2系列资料
│   ├── 2D  dm_orin_nx-cb-v3_1.pdf
│   ├── 3D  DM Orin NX-CB-V3.2.zip
│   ├── 3D文件
│   ├── Realtek_r8125_DKMS				# RTL-8125 驱动
│   ├── 外壳
│   ├── 网口转接板
│   └── 达妙科技DM-ORIN NX V2.X使用说明书V1.0.pdf
├── guest_source_or_other_information		# 客户方案和其他社区资料
│   ├── CAN功能测试
│   ├── USB3.0设备树修改
│   └── 客户自制外壳
├── orin载板can控制达妙电机例程		# 达妙 orin 载板 can 控制达妙电机例程
│   ├── dm_hw
│   ├── docs
│   ├── README.en.md
│   └── README.md
└── README.md

19 directories, 8 files
```

### 更新要求

1. 维护本仓库需要明确的的分类到具体的目标文件夹

2. 请您在每次更新后，重新更新2级目录的内容并对更新的条目和对新增文件进行说明

3. 仓库的内容尽可能做中英文两份


**Jetson Orin NX烧录+设备树更改？看这一篇就够了！：<https://blog.csdn.net/xiongqi123123/article/details/144079706>**

# 附：

1. P3767-0000这些神秘代码都是什么意思？

其中P3768-0000，就是你的载板，可以根据这些名字，找到对应的dts和dtsi文件，根据你的需求进行修改。

[Quick Start — NVIDIA Jetson Linux Developer Guide 1 documentation](https://docs.nvidia.com/jetson/archives/r35.5.0/DeveloperGuide/IN/QuickStart.html)

| Module                                 | Module Type                                 | Carrier Board                                         | Configuration           | Configuration Notes                                          |
| -------------------------------------- | ------------------------------------------- | ----------------------------------------------------- | ----------------------- | ------------------------------------------------------------ |
| Jetson Orin NX 16GB-DRAM (P3767-0000)  | Production                                  | Jetson Orin Nano reference carrier board (P3768-0000) | jetson-orin-nano-devkit | Flashes QSPI-NOR and USB/NVMe drive (only supported via l4t_initrd_flash.sh) |
| Jetson Orin NX 8GB-DRAM (P3767-0001)   | Production                                  | Jetson Orin Nano reference carrier board (P3768-0000) | jetson-orin-nano-devkit | Flashes QSPI-NOR and USB/NVMe drive (only supported via l4t_initrd_flash.sh) |
| Jetson Orin Nano 8GB-DRAM (P3767-0003) | Production                                  | Jetson Orin Nano reference carrier board (P3768-0000) | jetson-orin-nano-devkit | Flashes QSPI-NOR and USB/NVMe drive (only supported via l4t_initrd_flash.sh) |
| Jetson Orin Nano 4GB-DRAM (P3767-0004) | Production                                  | Jetson Orin Nano reference carrier board (P3768-0000) | jetson-orin-nano-devkit | Flashes QSPI-NOR and USB/NVMe drive (only supported via l4t_initrd_flash.sh) |
| Jetson Orin Nano 8GB-DRAM (P3767-0005) | Development（就是官方版开发套件，带sd卡的） | Jetson Orin Nano reference carrier board (P3768-0000) | jetson-orin-nano-devkit | Flashes QSPI-NOR and microSD Card/USB/NVMe drive (only supported via l4t_initrd_flash.sh) |























