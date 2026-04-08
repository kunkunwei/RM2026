# 云台地面端

> 基于 STM32F4 的云台控制项目。本 README 重点说明输入控制切换、云台-底盘通信、状态机、自瞄接口，以及工程重新生成注意事项。

## 1) 输入控制切换与硬件支持

### 支持的输入来源
- **遥控器（DBUS/SBUS）**
  - 通过编译选项选择协议（`USE_SBUS_PROTOCOL` 或默认 DBUS），见 `Application/Tasks/Inc/Gimbal_task.h`。
  - 模式切换逻辑在 `Application/Tasks/Src/Gimbal_task.c`（拨杆位置判断）。
- **PC 键鼠（图传链路 UART）**
  - 实现于 `Components/Device/Src/pc_uart_ctrl.c` 与 `Components/Device/Inc/pc_uart_ctrl.h`。
  - 使用 UART + DMA 双缓冲；默认串口由 `PC_CTRL_DEFAULT_HUART` 设置。
- **MiniPC 自瞄（USB CDC）**
  - USB CDC 接收入口：`Components/Device/Src/usb.c`。
  - 帧格式与解析：`Components/Device/Src/minipc.c`、`Components/Device/Inc/minipc.h`。

### 模式切换（概览）
- **RC（DBUS/SBUS）**：`Application/Tasks/Src/Gimbal_task.c` 中 `gimbal_mod` 的更新逻辑。
  - 右拨杆：**NO_FORCE** / **SLOW_CALI** / **NORMAL** / **FLOW_CHASSIS**。
  - 左拨杆：**AutoAim**。
- **PC（图传链路）**：`mode_sw`、`fn_1`、`fn_2`、`pause` 在 `Application/Tasks/Src/Gimbal_task.c` 中映射：
  - `mode_sw=0/1/2`：**NO_FORCE**、**SLOW_CALI**、**FLOW_CHASSIS**（带过渡判定）。
  - `fn_1`：切换 **AutoAim**。
  - `fn_2`：切换 **小陀螺**（spinning）模式。
  - `pause`：切换 **跳跃** 指令到地盘。

### 模式枚举
- `gimbal_mod_e` 位于 `Application/Tasks/Inc/Gimbal_task.h`：
  - `GIMBAL_MOD_NO_FORCE`、`GIMBAL_MOD_SLOW_CALI`、`GIMBAL_MOD_NORMAL`、`GIMBAL_MOD_AutoAim`、`GIMBAL_MOD_CONTROL_BY_PC`、`GIMBAL_MOD_FLOW_CHASSIS`。

## 2) 云台 <-> 底盘 通信控制

### 协议与传输
- **传输**：UART（USART6），固定帧长 20 字节。
- **协议定义**：`Components/Device/Inc/ctl_chassis.h`。
- **发送（云台 -> 底盘）**：`Components/Device/Src/ctl_chassis.c` 的 `gimbal_send_ctrl_cmd()`。
  - 字段：控制标志、yaw（缩放）、speed_x、speed_wz、腿长、roll、yaw_abs、yaw_gyro。
- **接收（底盘 -> 云台）**：`Components/Device/Src/ctl_chassis.c` 的 `gimbal_parse_feedback()`。
  - 反馈：chassis yaw、speed_x、speed_wz、length、yaw_rate、状态标志。

### 云台模式到地盘模式映射
- 映射逻辑在 `Components/Device/Src/ctl_chassis.c` 的 `gimbal_ctl_chassis_cmd()`。
- 对应 `chassis_mode_e`（`Application/Tasks/Inc/Gimbal_task.h`）：
  - `GIMBAL_MOD_NO_FORCE` / `GIMBAL_MOD_SLOW_CALI` -> `CHASSIS_FORCE_RAW`。
  - `GIMBAL_MOD_NORMAL` -> `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW`。
  - `GIMBAL_MOD_AutoAim` / `GIMBAL_MOD_FLOW_CHASSIS` -> `CHASSIS_VECTOR_NO_FOLLOW_YAW`。

## 3) 云台状态机与底盘状态机

### 云台控制状态机（主线）
- **主循环**：`Application/Tasks/Src/Gimbal_task.c`。
- `gimbal_control_loop()` 控制分支：
  - **NO_FORCE**：电机输出清零。
  - **SLOW_CALI**：慢速校准。
  - **小陀螺**：使用底盘反馈补偿。
  - **FLOW_CHASSIS**：云台跟随底盘。
  - **NORMAL / AutoAim / PC 控制**：常规跟踪控制。

### 底盘状态机
- 底盘控制由云台通过 `chassis_mode_e` 与控制标志驱动。
- 该仓库只包含云台侧协议与命令，底盘侧状态机代码不在本仓库内。

## 4) 自瞄接口

### USB CDC 自瞄输入（MiniPC）
- **帧格式**：`Components/Device/Inc/minipc.h` 的 `MiniPC_AutoAimFrame_Typedef`。
- **解码**：`Components/Device/Src/minipc.c` 的 `MiniPC_DecodeAutoAimFrame()`。
- **USB 接收入口**：`Components/Device/Src/usb.c` 的 `usbReceiveData()` / `usbReceive_autoAim_handle()`。
- **数据提供给云台**：
  - `Usb_AutoAim_t`（yaw、pitch、depth、control flags、shoot flag），通过 `getUsbMiniPcPtr()`。
- **与模式切换的关系**：
  - `Application/Tasks/Src/Gimbal_task.c` 会检查 `usb_autoAim_ptr->control_flag` 以进入/退出自瞄模式。

## 5) 重新生成项目（CubeMX）

如需用 STM32CubeMX 重新生成工程，建议流程如下以避免覆盖自定义构建配置：

1. **先在本地构建一次**（确保当前工程可用，便于回退）。
2. **在 CubeMX 重新生成工程**（可能覆盖 `CMakeLists.txt`）。
3. **回溯 `CMakeLists.txt` 为自定义版本**：
   - 用你保存的 `CMakeLists.txt` 覆盖生成文件。
   - 或参考 `CMakeLists_template.txt`，重新应用自定义 includes/defines/sources。

> 建议：在运行 CubeMX 前先备份 `CMakeLists.txt`。

---

### 快速文件索引
- 输入切换逻辑：`Application/Tasks/Src/Gimbal_task.c`
- 云台模式枚举：`Application/Tasks/Inc/Gimbal_task.h`
- PC 串口控制：`Components/Device/Src/pc_uart_ctrl.c`
- 自瞄 USB：`Components/Device/Src/usb.c`、`Components/Device/Src/minipc.c`
- 云台-底盘协议：`Components/Device/Src/ctl_chassis.c`
