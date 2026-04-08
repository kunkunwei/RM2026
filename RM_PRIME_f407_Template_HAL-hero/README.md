# RM_PRIME_f407_Template_HAL-hero

本项目是基于周州改的老麦轮英雄机器人代码整理而来，面向 STM32F407 + HAL + FreeRTOS 的英雄平台控制工程。主要包含云台/底盘/发射控制、裁判系统交互、自瞄接口与 UI 显示等模块。

## 1. 输入控制方式与硬件支持

### 1.1 遥控器 DBus（DJI 遥控器）
- 使用 `DBus` 读取通道与拨杆状态。
- 模式切换逻辑位于 `CppResource/Tasks/MotionControlTask/RemoteControlMidware.cpp` 的 `DBus_input()`。
- 拨杆组合映射：
  - 左拨杆 `s[1]==1` -> `IntoIdle`
  - 左拨杆 `s[1]==3` -> `IntoChassisLead`
  - 左拨杆 `s[1]==2 && 右拨杆 s[0]==3` -> `IntoGimbalLead`
  - 左拨杆 `s[1]==2 && 右拨杆 s[0]==1` -> `IntoAutoAim`
  - 左拨杆 `s[1]==2 && 右拨杆 s[0]==2` -> `IntoAutoRotate`
- 通道映射：
  - `ch[3]` 前后速度
  - `ch[2]` 横移速度
  - `ch[0]` 底盘转向/云台引导
  - `ch[1]` 俯仰

### 1.2 VT03 键鼠（图传/键鼠模块）
- 使用 `VT03` 读取键鼠与拨盘状态。
- 键位映射位于 `VT03_input()`：
  - `Q`：小陀螺（AutoRotate）开关
  - `E`：无力模式（Idle）切换
  - `Z`：ChassisLead
  - `X`：GimbalLead
  - `C`：AutoAim
- 键鼠在线时，忽略遥控器拨杆切换（以键鼠优先）。

### 1.3 裁判系统与电源状态
- `RemoteContrlMidwareLoop()` 会读取裁判系统电源状态：
  - `power_management_gimbal_output`
  - `power_management_chassis_output`
- 断电时触发 `PowerOff`，进入 `Init` 状态；上电时重新允许进入工作状态。

## 2. 云台与底盘的关联控制

- 模式切换后，云台与底盘通过状态机共享姿态与速度信息。
- 在 `GimbalLead` / `AutoAim` / `AutoRotate` 中，底盘速度在云台坐标系与世界坐标系之间进行旋转变换：
  - 参考 `GimbalLeadLoop.cpp`、`AutoAimLoop.cpp`、`AutoRotateLoop.cpp`。
- 底盘跟随云台偏角：
  - 当云台偏角超过 `CHASSIS_FOLLOW_DES` 时，底盘会产生 `CHASSIS_FOLLOW_OMEGA` 的跟随角速度。
- 云台 yaw 在各模式中由不同的参考生成：
  - ChassisLead：保持 `YawZero`。
  - GimbalLead：由云台相对旋转积分得到。
  - AutoAim：由自瞄输入角度增量驱动。
  - AutoRotate：小陀螺模式下持续旋转。

## 3. 云台/底盘状态机控制

状态机定义在 `CppResource/Tasks/MotionControlTask/MotionFSM/MotionFSM.h`。

### 3.1 状态
- `Init`：上电初始化态
- `Idle`：无力
- `ChassisLead`：底盘主导
- `GimbalLead`：云台主导
- `AutoAim`：自瞄
- `AutoRotate`：小陀螺

### 3.2 事件与转换
- `InitComplete`：上电就绪 -> `Idle`
- `Into*`：直接切换目标模式
- `PowerOff`：电源断开 -> `Init`

状态切换由 `RemoteContrlMidwareLoop()` 输入事件驱动；每个状态使用对应的 `*Loop.cpp` 完成控制计算。

## 4. 自瞄接口

- 自瞄数据由 `NanoMsg` 提供。
- `AutoAimLoop.cpp` 中通过 `NanoMsg::getControlCmd()` 获取 yaw/pitch 角度指令。
- 自瞄控制融合当前姿态与 IMU 角度：
  - yaw：`target = cur + aim_yaw - imu_yaw`
  - pitch：`target = cur + aim_pitch - imu_pitch`
- 接口文件：
  - `CppResource/Device/NanoMsg.h`
  - `CppResource/Device/NanoMsg.cpp`

## 5. 其它补充

### 5.1 目录说明（核心）
- `CppResource/Tasks/MotionControlTask/`：运动控制与状态机
- `CppResource/Tasks/ShootTask/`：发射控制与摩擦轮
- `CppResource/Device/`：各类设备驱动（IMU/裁判系统/遥控器等）
- `UI_bismarckkk/`：**UI 成功案例（非本项目代码）**，仅供参考
- `CppResource/Tasks/ClientUI_Task.cpp`：本项目 UI 相关逻辑

### 5.2 构建与烧录（CMake）
- CMake + GCC ARM 工具链，工程已包含 `CMakePresets.json`。
- 典型流程：
  1. 生成工程
  2. 编译
  3. 使用 OpenOCD 或 ST-Link 下载

### 5.3 调试与参数
- 参数集中在 `MotionControl.h` 的 `MotionParameter`。
- 调整底盘跟随、云台 KP/KD、最大速度等均可在此完成。

### 5.4 UI 显示
- 本项目 UI 通过串口与裁判系统交互绘制。
- 若需要参考完整流程，请查看 `UI_bismarckkk/` 中的案例。

### 5.5 已知风险与排查建议
- 上电后未进入可控状态：检查裁判系统电源状态是否有效。
- 云台/底盘不跟随：检查 `CHASSIS_FOLLOW_*` 参数与 IMU 角度是否正常。
- 自瞄无效：确认 `NanoMsg` 数据链路是否正常、角度坐标系是否一致。

### 5.6 无裁判系统（裁判系统被拆下）
当无法接入裁判系统时，需要关闭功率限制并让状态机不再依赖裁判系统电源状态，否则会一直处于 `Init` 或 `PowerOff`。

- 功率限制关闭（建议固定参数）
  - 位置：`CppResource/Tasks/MotionControlTask/ChassisControl.cpp`
  - 逻辑：`ChassisControl::setMove()` 里只有在 `RefereeExist()` 时才使用裁判系统功率数据。
  - 做法：在无裁判系统模式下，强制使用固定的 `power_buffer`/`power_limit`，并将 `power_on = true`，使超电保持工作。

- 电源状态与状态机关联（无裁判系统时默认上电）
  - 位置：
    - `CppResource/Tasks/MotionControlTask/RemoteControlMidware.cpp`（电源开关与 `PowerOff` 事件）
    - `CppResource/Tasks/MotionControlTask/MotionFSM/InitLoop.cpp`（`Init` -> `Idle`）
  - 逻辑：当前实现依赖 `power_management_*_output`，无裁判系统时会一直认为断电。
  - 做法：当 `RefereeExist()` 为 `false` 时，强制 `gimbal_on = true`、`chassis_on = true`，让状态机正常进入 `Idle` 并响应遥控。

- 建议开关（可选）
  - 增加一个编译期宏或运行时标志（例如 `NO_REFEREE_MODE`），统一控制上述两个逻辑分支，便于比赛/调试切换。

