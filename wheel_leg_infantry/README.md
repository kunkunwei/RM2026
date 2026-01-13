# Wheel-Leg Infantry Robot Control Firmware (轮足步兵机器人底盘控制)

## 项目简介
本项目是为五连杆并联腿轮足机器人（RoboMaster 步兵机器人）开发的底盘控制固件。
采用 **STM32F407IGH6** 作为主控制器，基于 **FreeRTOS** 实时操作系统和 **HAL库** 进行开发。

核心控制算法采用 **LQR (Linear Quadratic Regulator)**，通过离线计算反馈矩阵并插值求解，相比MPC等算法，实现了极低资源占用的快速稳定平衡控制。

## 硬件平台
- **MCU**: STM32F407IGH6 (ARM Cortex-M4, 168MHz)
- **构型**: 五连杆并联腿 + 轮足 (Diamond/V-shape linkage)
- **电机**:
  - 关节电机 x 4 (左右各2)
  - 轮毂电机 x 2 (足端)

## 软件架构
- **开发环境**: STM32CubeMX, Keil/CLion (CMake)
- **操作系统**: FreeRTOS
- **驱动库**: STM32 HAL Library

### 核心算法
1. **平衡控制 (LQR)**
   - 采用离线 LQR 计算方案，预先计算不同腿长下的最优反馈矩阵 $K$。
   - 运行时根据当前腿长插值获取 $K$ 矩阵，计算控制量 $u = -Kx$。
   - 状态量 $x$: [$\theta$, $\dot{\theta}$, $x$, $\dot{x}$, $\phi$, $\dot{\phi}$] (摆杆角度，摆杆角速度，位移，速度，机体角度，机体角速度)
   
2. **运动学解算**
   - 五连杆逆运动学解算：将虚拟腿长和关节角度映射到两个关节电机的角度。
   - VMC (Virtual Model Control) 思想：将足端力映射为关节力矩。

3. **抗干扰与打滑处理**
   - 包含针对打滑检测的逻辑，当检测到打滑或碰撞时，通过额外的补偿力矩迅速恢复稳定。

## 快速上手

### 1. 代码目录结构
- **Application/**: 应用层代码
  - `Tasks/`: FreeRTOS 任务
    - `chassis_task.c`: 底盘控制主循环 (2ms/500Hz)
    - `chassis_behaviour.c`: 底盘行为状态机 (无力、起立、跟随云台、自旋等)
  - `API/`: 应用接口
- **Bsp/**: 板级支持包 (CAN, SPI, UART, PWM等)
- **Components/**: 组件层
  - `Algorithm/`: 算法实现
  - `Controller/`: 控制器实现 (PID等)
- **Matlab_lib/**: MATLAB 自动生成的代码 (LQR增益表、运动学公式)

### 2. 关键控制接口
主要控制逻辑在 `chassis_behaviour.c` 中配置。常用的控制指令变量：
- `vx_set`: 前进速度设定值 (m/s)
- `l_set`: 腿长设定值 (m)，即虚拟摆杆长度
- `angle_set`: 角度设定值 (rad)，用于偏航转向或姿态微调

### 3. 调试说明
- **调试接口**: 使用 J-Link 或 ST-LinkSWD 接口。
- **数据监控**: 推荐使用 Vofa+ 或类似串口示波器监控 `chassis_task` 中的状态变量。

## 更新日志
- **2026-01-12**: 修正底盘行为控制函数的注释，明确了五连杆机器人的控制参数（速度、腿长、角度）。

