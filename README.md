# DM-Orin 载板资料入口

## 概述
- 这里是 DM-Orin 载板资料入口，按 `public information`、`v1/v2` 版本资料、客户/其他资料和 `CAN` 例程分组。
- 本页保留了必要的操作说明和维护流程，不再直接删掉原有流程性内容。
- 维护时先确认当前板卡版本，再进入对应资料目录；`P3767` / `P3768` 这类编号可用于定位匹配的 `dts` / `dtsi` 文件。

## 文档 / 资源
- [dm_orin_board_public_information/](dm_orin_board_public_information/)
- [dm_orin_board_v1_information/](dm_orin_board_v1_information/)
- [dm_orin_board_v2_information/](dm_orin_board_v2_information/)
- [guest_source_or_other_information/](guest_source_or_other_information/)
- [orin载板can控制达妙电机例程/](orin载板can控制达妙电机例程/)
- [CAN 功能测试.md](CAN 功能测试.md)
- [Jetson Orin NX烧录+设备树更改？看这一篇就够了！](https://blog.csdn.net/xiongqi123123/article/details/144079706)
- [NVIDIA Jetson Linux Developer Guide - Quick Start](https://docs.nvidia.com/jetson/archives/r35.5.0/DeveloperGuide/IN/QuickStart.html)
- [Orin NX-RJ45-V1.0.pdf](Orin NX-RJ45-V1.0.pdf)
- [rtc设置.rar](rtc设置.rar)

## 快速开始
- 先确认你的板卡版本，再进入 `dm_orin_board_v1_information/` 或 `dm_orin_board_v2_information/`。
- 如果要做烧录、设备树修改、CSI 相机适配、UART 使能或无线网卡驱动处理，先查看 `dm_orin_board_public_information/` 和 `guest_source_or_other_information/` 中的对应资料。
- 如果要修改 `P3767-0000`、`P3768-0000` 这类板卡编号，请先参考 NVIDIA Quick Start，再定位对应的 `dts` / `dtsi` 文件。
- 每次更新资料后，先刷新二级目录索引，再补充新增条目说明。
- 不能确认的内容统一写成 `TBD` 或 `Translation pending`，不要硬补。

## 状态
- ZH: 主版
- EN: Translation pending
- TBD: 未确认信息保留空缺
