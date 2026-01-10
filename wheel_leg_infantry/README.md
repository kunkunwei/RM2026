# wheel_leg_infantry 项目说明


## 1. 项目概述

该项目针对基于 STM32F407 的嵌入式控制器，采用 STM32 HAL 驱动库、FreeRTOS 以及基于 CMake 的构建系统。项目包含底层 MCU 外设封装（Core、Drivers、Bsp）、设备启动代码（Startup）、USB 设备中间件（USB_DEVICE）、若干控制/算法模块（Components/Algorithm）和 Matlab 自动生成/移植的库（Matlab_lib）。

本仓库的目标是实现轮—腿机器人（wheel-leg）相关控制算法与驱动代码，并提供可直接构建的工程配置（已包含 cmake-build-debug 下的构建产物示例）。

---

## 2. 关键文件与目录说明

仓库顶层重要项（相对仓库根目录）说明：

- `CMakeLists.txt` / `CMakeLists_template.txt`：CMake 构建脚本与模板，用于生成 Makefile/工程文件。
- `wheel_leg_infantry.ioc`：STM32CubeMX 工程文件（外设配置、时钟、引脚、FreeRTOS 配置等），修改硬件配置请先用 STM32CubeMX 打开并重新生成代码。
- `cmake-build-debug/`：本地 IDE（如 CLion）或 CMake 生成的构建目录，包含 `.elf/.bin/.hex` 等产物（示例：`wheel_leg_infantry.elf`, `wheel_leg_infantry.bin`, `wheel_leg_infantry.hex`）。

主要源码目录：

- `Core/`：MCU 相关核心代码
  - `Core/Src/main.c`：系统入口，任务初始化与调度入口点。
- `Application/`:FreeRtos实际运行的任务与自瞄决策模块
  -`Application/Tasks/Src/`:FreeRtos任务实现文件，开发与调试主要看这部分内容，里面包含了整机控制代码和状态监控、上位机通信等功能模块。
  -`Application/API`:对外提供的接口函数，其中有对上位机发送数据的结构体（包括云台的角度、装甲板识别信息） 
- `Drivers/`、`Drivers/STM32F4xx_HAL_Driver/`：STM32 HAL 驱动及 CMSIS 文件。

- `Bsp/`：板级支持包（GPIO、UART、TIM、I2C、SPI、CAN 等封装），常用接口在 `Bsp/Inc`。

- `Components/Algorithm/`：项目的控制与算法实现（滤波、校验、卡尔曼滤波、打滑检测、theata角度微分预测等）。

- `Matlab_lib/`：由 MATLAB/Simulink 生成/移植的函数和头文件（例如 `Matlab_lib/inc/leg_spd.h`、`leg_spd.c` 等），用于实现轮腿运动学解算、足端角速度解算、VMC力矩计算等。

- `USB_DEVICE/` / `Middlewares/`：USB Device 中间件与示例实现（CDC/CDC-IF），STM32cubemx 生成。

在调试或修改逻辑时，常查看的文件：`Application/Tasks/Src`、`CMakeLists.txt`、`wheel_leg_infantry.ioc`、`Components/Algorithm/` 下的各模块。

---

## 3. 准备工作（依赖）

建议在 Windows 环境 (PowerShell) 下准备以下工具：

- CMake（建议 >= 3.15）
- GNU Make 或 Ninja（取决于 CMake 生成器），在 Windows 下可通过 MSYS2、MinGW 或 WSL 安装
- ARM 嵌入式工具链：`arm-none-eabi-gcc` / `arm-none-eabi-binutils` / `arm-none-eabi-gdb`
- OpenOCD（用于通过 SWD 调试/刷写）或 ST 官方工具（`ST-Link_CLI` 或 ST-Link 驱动）
- 可选：`st-flash`（来自 `stlink` 工具集）用于通过 USB-STLink 刷写

安装建议：若使用 MSYS2 或类似环境，请确保把 `arm-none-eabi-*` 二进制加入 PATH，CMake 与 Make 也在 PATH 中。

---

## 4. 快速构建（PowerShell 示例）

以下示例在 PowerShell 中运行，假设你当前工作目录为项目根目录（包含 `CMakeLists.txt`）：

1) 创建构建目录并生成 Makefile（或 Ninja）：

```powershell
# 在项目根目录下
mkdir build; cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
# 如果你没有 toolchain.cmake，可以直接让 CMake 使用系统 PATH 下的 arm-none-eabi 工具链
```

2) 构建项目：

```powershell
# 使用 Make
cmake --build . -- -j 4
# 或直接
make -j 4
```

3) 构建产物位置

构建完成后，通常在 `build/` 或 `cmake-build-debug/` 下可找到生成的 ELF/BIN/HEX 文件，例如：

- `wheel_leg_infantry.elf`
- `wheel_leg_infantry.bin`
- `wheel_leg_infantry.hex`

如果你使用 IDE（如 CLion），IDE 会在 `cmake-build-debug/` 下生成对应产物（仓库中已包含示例产物）。

注意：项目可能包含特定的 toolchain 文件或 CMake 选项；请查看 `CMakeLists.txt` 或项目根目录中是否有 `toolchain` 相关文件以获得更准确的构建参数。

---

## 5. 刷写固件到目标板（示例命令，PowerShell）

以下示例展示常见工具的刷写方法。根据你的硬件连接（ST-Link / DAPLink / OpenOCD 支持的适配器）选择合适命令。

- 使用 OpenOCD（假设系统已安装 openocd 并能识别 ST-Link）：

```powershell
# 启动 OpenOCD（示例，适配你的配置文件路径）
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
# 在另一个 PowerShell 窗口使用 telnet 或 openocd 的命令行接口刷写
# 示例：通过 telnet (127.0.0.1:4444)
# > telnet 127.0.0.1 4444
# > reset halt
# > flash write_image erase ..\cmake-build-debug\wheel_leg_infantry.bin 0x08000000
# > reset run
```

- 使用 st-flash（来自 stlink 工具集）：

```powershell
st-flash write .\cmake-build-debug\wheel_leg_infantry.bin 0x08000000
```

- 使用 ST-Link_CLI（ST 官方工具）：

```powershell
# 示例（根据安装路径调整）
& 'C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK_CLI.exe' -c SWD -P .\cmake-build-debug\wheel_leg_infantry.bin 0x08000000 -V
```

提示：不同工具的地址基址（上例为 `0x08000000`）以链接脚本中定义的 FLASH 起始地址为准（请查看工程中的链接脚本，如 `STM32F407IGHX_FLASH.ld`）。

---

## 6. 调试（GDB + OpenOCD）

1) 启动 OpenOCD：

```powershell
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
```

2) 使用 arm-none-eabi-gdb 连接（示例）：

```powershell
arm-none-eabi-gdb .\cmake-build-debug\wheel_leg_infantry.elf
# 在 gdb 提示符下
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

---

## 7. Components/Algorithm 下开发建议

- 目录用途：`Components/Algorithm/` 存放控制算法、轨迹规划、滤波、状态机等逻辑模块。修改此处代码后，按常规步骤重新构建并刷写固件验证。

- 查找函数与头文件：若需要定位某个符号（例如 `leg_spd`），仓库中 `Matlab_lib/inc/leg_spd.h` 提供了函数声明，实际实现可能在 `Matlab_lib/src/leg_spd.c` 或 `Components/Algorithm` 下的相应源文件。使用 IDE 的全局搜索（或命令行 `grep` / `rg`）快速定位实现。

- 专用建议：
  - 保持模块化：在 `Components/Algorithm/` 中按功能拆分文件（例如 `kinematics.c`、`trajectory.c`、`controller.c`）。
  - 单元测试：由于嵌入式代码难以在主机直接运行，建议把纯算法（无 HAL 依赖）的函数抽离出来，编写 host-side 单元测试（使用 CMake 的 `add_executable` 针对 host 构建，或使用 CMocka/Unity 等测试框架）。
  - 仿真调试：可以在 PC 上用替代实现（stub HAL 接口）或使用 QEMU（若支持）进行快速回归测试。

---

## 8. 常见问题与排查建议

- 构建找不到 `arm-none-eabi-gcc`：检查工具链是否在 PATH 中，或是否需要在 CMake 中指定 `CMAKE_TOOLCHAIN_FILE`。
- 链接错误（undefined reference）：检查是否把对应的源文件加入了 CMakeTargets，或是否漏编译某个模块。
- 丢失头文件：确认 include 路径（CMake 中的 include_directories）、以及文件名大小写是否一致（Windows 不区分大小写但目标系统/工具链有时敏感）。
- OpenOCD 无法识别设备：确认 ST-Link 驱动安装正确，并且使用正确的 OpenOCD interface/target 配置文件。

调试技巧：使用 `cmake --build . -- VERBOSE=1` 或 `make VERBOSE=1` 查看完整的编译命令，用于诊断编译器参数或链接器脚本问题。

---

## 9. 查找与修改示例（以 `leg_spd` 为例）

- 声明在：`Matlab_lib/inc/leg_spd.h`
- 实现可能在：`Matlab_lib/src/` 下的相关 `leg_spd.c` 或 `Components/Algorithm/` 下的实现文件

修改流程示例：
1. 在 IDE 中打开 `Matlab_lib/inc/leg_spd.h` 并阅读注释与参数说明；
2. 修改 `Matlab_lib/src/leg_spd.c`（或相关实现文件）；
3. 重新构建并刷写固件验证结果：遵循第 4、5 节说明。

---

## 10. 贡献与许可证（占位）

欢迎贡献：请使用 Feature 分支，提交 Pull Request，保持单个 PR 关注单一功能/修复；描述清楚变更理由与测试步骤。

许可证：本 README 处放置占位说明；请项目所有者在仓库根目录添加 `LICENSE` 文件并在此处说明（例如 MIT / Apache-2.0 / GPL）。

---

## 11. 附录：有用的命令汇总（PowerShell）

# 创建构建目录并生成
mkdir build; cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build . -- -j 4

# 刷写（st-flash）
st-flash write ..\cmake-build-debug\wheel_leg_infantry.bin 0x08000000

# OpenOCD + GDB（快速示例）
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
arm-none-eabi-gdb ..\cmake-build-debug\wheel_leg_infantry.elf

---

感谢使用本项目！如有疑问或建议，请提交 Issue 或联系维护者。祝开发顺利！