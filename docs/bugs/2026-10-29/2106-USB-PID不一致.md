---
id: R2
title: USB PID 不一致【严重，已修】
severity: 严重
status: 已修
date: 2026-10-29 21:06
commit: 5a787c1
---

# R2 USB PID 不一致【严重，已修】

> 严重度：严重 ｜ 状态：已修 ｜ 提交：`5a787c1` ｜ 日期：2026-10-29 21:06

固件 `USB_DEVICE/App/usbd_desc.c`：`USBD_PID_FS = 22337 = 0x5741`；上位机写死 `0x5740`
（ST 官方模板值）。`libusb_open_device_with_vid_pid` 找不到设备。已改为 `0x5741`。

## 相关条目

- [R1](../2026-10-29/2106-通信帧与固件不兼容.md)
- [INV8](../2026-10-29/2106-实机联调完整记录.md)
