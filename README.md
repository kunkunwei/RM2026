# RM2026

2025-2026赛季开发代码仓库。

- 仓库：`kunkunwei/RM2026`
- 默认分支：`master`
- 主要语言：C / C++

## 目录结构（概览）

> 下面是仓库根目录下的主要工程/模块文件夹（以实际内容为准）：

- `fw/`：张大头步进电机
- `gimbal_ground/`：轮腿云台项目
- `gimbal_fly/`：无人机云台项目
- `new_hero/`：麦轮英雄项目，手搓之后没有完全测试，勉强能用
- `wheel_leg_infantry/`：轮腿步兵项目，完成平衡、跳跃、打滑补偿功能，还需补充小陀螺前进功能。
- `My_NewChassis/`：手搓的新底盘项目，没啥用
- `Feibiao_V1/`：飞镖项目（V1），临时搓的，估计BUG一堆。
- `RM_PRIME_f407_Template_HAL-hero/`：周州重新搓的英雄，后来我再魔改的屎山
- `Wheel-legged-robot-MATLAB-simulation-collection-main/`：轮腿机器人 MATLAB 仿真/资料集合
- `上位机/`：达妙、瓴控上位机相关（可视化、调参、通信工具等）
- `Sentinel_Gimbal/`：哨兵云台

## 开发环境建议

> 本仓库包含多个工程，可能分别对应不同硬件/编译环境。以下为常见建议，请按具体工程的说明调整。

### 嵌入式/固件（STM32 等）
- IDE：
  - clion +  CMake
  - VS Code + CMake
- 工具链：
  - ARM GCC（如 `arm-none-eabi-gcc`）
- 调试下载：
  - ST-Link / J-Link（视硬件而定）

### 上位机
- 请进入 `上位机/` 查看

### 仿真（MATLAB）
- 进入 `Wheel-legged-robot-MATLAB-simulation-collection-main/`，使用 MATLAB 打开对应脚本/工程运行。

## 如何开始

1. 先确定你要使用的项目（例如：`new_hero/`、`gimbal_ground/`、`wheel_leg_infantry/` 等）。
2. 进入该目录，查找以下任一类文件（若存在）：
   - `README.md` / `docs/`
   - `*.ioc`（CubeMX 工程入口）
   - `*.uvprojx`（Keil 工程入口）
   - `CMakeLists.txt`（CMake 工程入口）
3. 按工程要求配置编译器、链接脚本、外设库与下载方式。


## 分支与提交规范（建议）

- 功能分支：`feat/<module>-<desc>`
- 修复分支：`fix/<module>-<desc>`
- 提交信息示例：
  - `feat(gimbal): add imu calibration`
  - `fix(chassis): resolve can timeout`

## 贡献方式

欢迎提 Issue / PR：
1. Fork 或新建分支
2. 提交改动并自测
3. 发起 PR，描述改动内容与影响范围

## License

未指定开源协议（）。
