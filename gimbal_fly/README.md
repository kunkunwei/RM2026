# gimbal_fly

无人机三轴云台控制代码（快速开发版）。云台控制本质与地面云台一致，但无人机云台的 YAW 不需要 360 度连续转动。

## 控制输入与切换

项目支持两类输入：遥控器（RC）与串口（USART）。切换通过编译宏控制。

- 遥控器模式：`USE_RC_CONTROL`
- 串口模式：`USE_USART_CONTROL`
- 遥控协议：默认 DBUS，定义 `USE_SBUS_PROTOCOL` 则启用 SBUS
- 使用 SBUS 时需在 STM32CubeMX 将 USART3 配置为 `100K, 8E2`
- 使用 DBUS 时需在 STM32CubeMX 将 USART3 配置为 `100K, 8N1`

遥控器通道与拨杆定义在 `Application/Tasks/Inc/Gimbal_task.h`。常用含义：

- YAW/PITCH/ROLL 由摇杆通道控制，灵敏度由 `GIMBAL_*_RC_SEN` 设置
- SBUS 与 DBUS 的通道和拨杆编号不同，请参考宏区

## 云台状态机

云台模式（`gimbal_mod_e`）状态机在 `gimbal_set_mod()` 中完成切换：

- `GIMBAL_MOD_NO_FORCE`：无力输出，电机不驱动
- `GIMBAL_MOD_SLOW_CALI`：慢速校准
- `GIMBAL_MOD_NORMAL`：正常遥控控制
- `GIMBAL_MOD_AutoAim`：自瞄控制
- `GIMBAL_MOD_AutoBalance`：ROLL 自稳模式
- 其它模式保留（`GIMBAL_MOD_SHAKING`、`GIMBAL_MOD_CONTROL_BY_PC`）

模式切换时目标与状态同步由 `gimbal_mod_change_date_transfer()` 处理。

发射系统有独立状态机（`shoot_mode_e`），在 `gimbal_set_control()` 中驱动并在 `cal_gimbal_shoot_control()` 中执行：

- `SHOOT_STOP` → `SHOOT_READY_FRIC` → `SHOOT_READY`
- `SHOOT_BULLET` 单发
- `SHOOT_CONTINUE_BULLET` 连发

包含拨弹轮堵转检测与反转解卡逻辑（`trigger_motor_turn_back()`）。

## 自瞄接口（MiniPC/USB）

自瞄数据结构位于 `Components/Device/Inc/usb.h`：

```c
typedef struct {
    float minipc_target_yaw;
    float minipc_target_pitch;
    uint8_t User;
    uint8_t crc;
    bool shoot_flag;
    uint8_t autoMod;
} Usb_AutoAim_t;
```

使用 `getUsbMiniPcPtr()` 获取指针，控制逻辑在 `minipc_control()`。

- 当 `minipc_target_yaw` 或 `minipc_target_pitch` 为 NaN 时视为无效
- YAW 直接覆盖目标位置，PITCH 以增量方式平滑逼近

## 重新生成项目（STM32CubeMX）

如果需要重新生成项目：

1. 先在本地完成一次构建，确保当前工程可编译。
2. 再使用 STM32CubeMX 重新生成工程。
3. 生成后，将 `CMakeLists.txt` 回溯到你已有版本（建议以 `CMakeLists_template.txt` 作为参考基线）。

注意：CubeMX 会覆盖部分工程文件，务必检查 `CMakeLists.txt` 与关键配置是否被重置。

## 重要说明与建议

- 本项目为快速开发版，鲁棒性较差；实际使用需加强异常处理与保护逻辑。
- 可参考 `gimbal_ground` 项目做完整实现与结构化重构。
- 无人机云台 YAW 不需要 360 度连续旋转，相关控制逻辑与地面云台略有差异。

