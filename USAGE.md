# DM-Orin 载板使用说明

## 概述
- 本文档收纳从 README 迁移出来的烧录、设备树、CSI、UART、Wi-Fi、CAN 以及维护提示。
- 资料仍按 `public information`、`v1/v2` 版本资料、客户/其他资料和 `CAN` 例程分组。

## 目录快照
- `dm_orin_board_public_information/`：通用教程目录，适用于 v1、v2 载板。
- `dm_orin_board_v1_information/`：DM-Orin V1 系列资料。
- `dm_orin_board_v2_information/`：DM-Orin V2 系列资料。
- `guest_source_or_other_information/`：客户方案和其他社区资料。
- `orin载板can控制达妙电机例程/`：载板 CAN 控制达妙电机例程。
- `CAN 功能测试.md`：CAN 功能测试说明。

## 使用流程
1. 先确认你的板卡版本，再进入对应的 `v1` 或 `v2` 资料目录。
2. 如果要做烧录、设备树修改、CSI 相机适配、UART 使能或无线网卡驱动处理，先查看 `dm_orin_board_public_information/` 和 `guest_source_or_other_information/` 中的对应资料。
3. 如果要修改 `P3767-0000`、`P3768-0000` 这类板卡编号，请先参考 NVIDIA Quick Start，再定位对应的 `dts` / `dtsi` 文件。
4. 如果要做 CAN 相关验证，先看 `orin载板can控制达妙电机例程/README.md` 和 `CAN 功能测试.md`。
5. 每次更新资料后，先刷新二级目录索引，再补充新增条目说明。

## 参考链接
- [Jetson Orin NX烧录+设备树更改？看这一篇就够了！](https://blog.csdn.net/xiongqi123123/article/details/144079706)
- [NVIDIA Jetson Linux Developer Guide - Quick Start](https://docs.nvidia.com/jetson/archives/r35.5.0/DeveloperGuide/IN/QuickStart.html)
- [Orin NX-RJ45-V1.0.pdf](Orin NX-RJ45-V1.0.pdf)
- [rtc设置.rar](rtc设置.rar)

## 更新要求
1. 维护本仓库时，必须明确分类到具体目标文件夹。
2. 每次更新后，重新更新二级目录内容，并对更新条目和新增文件进行说明。
3. 仓库内容尽可能保留中英文两份，但未确认内容不得硬补。
4. 不能确认的内容统一写成 `TBD` 或 `Translation pending`。
