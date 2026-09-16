# Circle_Leg_V1 — REACHABLE（QuadCirc）半尺寸原型固件

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32G473-03234B?logo=stmicroelectronics&logoColor=white" alt="STM32G473">
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-5CB85C" alt="FreeRTOS">
  <img src="https://img.shields.io/badge/C%2B%2B-GNU%2B%2B14-00599C?logo=cplusplus&logoColor=white" alt="C++">
  <img src="https://img.shields.io/badge/Control-500%20Hz-orange" alt="500 Hz">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License">
</p>

<p align="center">
  <a href="README.md">English</a> | <b>中文</b>
</p>

<p align="center">
  <img src="docs/figures/climbing_sequence.jpg" width="760" alt="半尺寸原型翻越台阶">
</p>

REACHABLE（QuadCirc）**半尺寸概念验证原型** 的嵌入式控制固件。CircLeg 偏心轮腿、变阻抗主动悬挂和
力矩残差台阶攀爬，都是在这台原型上首次实现并验证的。香港科技大学毕业设计项目 **SL05a-25**。

> 最终的全尺寸整车沿用同一套控制架构，只是换了电机和几何参数，固件见
> [Circle_Leg_V2.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0)。

---

## 项目概述

半尺寸原型验证的是这个项目最核心的想法：**轮子本身就是腿**。每个角有一个轮电机和一个腿电机，
腿电机绕偏心距转动轮毂，即可升降该角，从而让底盘自己调平，并翻越台阶边缘。

- **硬件**：STM32G473 + FreeRTOS（500 Hz）、4 个 DJI M3508 轮电机、4 个工作在 MIT 阻抗模式的 HT8115
  腿电机（编译选项仍保留 GM6020）、ICM-42688-P IMU，以及与上位机的串口链路。
- **控制**：梯形底盘的差速逆运动学、轮腿解耦、Pitch/Roll 调平、变阻抗悬挂、Warp 补偿，以及带力矩残差
  接触检测的逐腿运动学攀爬。
- **带到 V2 的经验**：最初一版用 GM6020 作腿电机，负载下抬不起底盘。换成 HT8115（扭矩约 10 倍、原生
  支持 MIT 阻抗控制）之后，柔顺悬挂这条路才走得通。

---

## 演示

<p align="center">
  <a href="https://youtu.be/onJCvx1d8Sw">
    <img src="docs/figures/demo_video.jpg" width="640" alt="REACHABLE 项目视频（YouTube）">
  </a>
</p>

<p align="center">
  ▶ <a href="https://youtu.be/onJCvx1d8Sw">在 YouTube 观看 REACHABLE 项目视频</a>
</p>

---

## 主要结果

半尺寸验证，数据引自团队最终项目报告：

| 项目 | 结果 |
|------|------|
| 行驶速度 | 约 1.34 m/s，行驶与转向可控 |
| CircLeg 驱动 | MIT 阻抗模式下全行程平顺 |
| 台阶攀爬 | 小路沿最高约 6.5 cm，受限于 65 mm 的偏心距 |
| 测试条件 | 75 cm 假人，测试流程参照 ISO 7176 的稳定性与越障测试 |

所有控制算法都以 *R*、*r* 和台阶高度作为参数，因此同一套代码可以直接放大到全尺寸几何；更大的偏心距
正好覆盖 5–18 cm 的目标范围。

<p align="center">
  <a href="https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0">
    <img src="docs/figures/v2_fullsize.jpg" width="220" alt="全尺寸后继车型 Circle_Leg_V2">
  </a>
</p>
<p align="center"><sub>采用同一控制架构的全尺寸整车 →
<a href="https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0">Circle_Leg_V2.0</a></sub></p>

---

## 系统架构

<p align="center">
  <img src="docs/figures/hw_architecture.svg" width="760" alt="半尺寸硬件架构">
</p>

```
Host PC ──UART 2 Mbit/s──► PC_Comm ──► Chassis_Task (500 Hz)
                                          │
                                   Chassis state machine
                  CALIBRATION · IDLE · ENERGY_SAVING · COMFORT · CLIMBING · FREE_CONTROL · DEBUG
                                          │
          Impedance_Controller · Ground_Contact (warp) · Climbing_Dynamics · Body PID · IK
                                          │
                               Wheel_Leg ×4 (FL · FR · BL · BR)
                          ┌───────────────┴────────────────┐
                    CAN: 4× M3508 (C620)             CAN: 4× HT8115 (MIT)
                    wheel velocity PID               leg {θ, ω, Kp, Kd, τ_ff}
```

控制链路图（HTML，下载后用浏览器打开）：
[COMFORT](docs/control-chains/Comfort_Mode_Chain.html) ·
[CLIMBING](docs/control-chains/Climbing_Mode_Chain.html)。

---

## 控制设计

### 偏心运动学

*R* = 160 mm，*r* = 65 mm。V1 采用的约定是：

$$H(\theta) = R + r\cos\theta, \qquad \theta = \arccos\frac{H - R}{r}, \qquad \left|\frac{\partial H}{\partial\theta}\right| = r\,|\sin\theta|$$

工作范围被限制在 [10°, 170°]，以远离 0° 和 180° 的奇异点。每个角的弯曲方向（向外或向内）可以单独配置。

### 行驶

<p align="center">
  <img src="docs/figures/skid_steer_kinematics.png" width="420" alt="梯形底盘差速运动学">
</p>

底盘为梯形（*W<sub>F</sub>* = 531 mm，*W<sub>B</sub>* = 395 mm，轴距 270 mm）。前后轴各自的增益
$k_F = W_F/W_\text{avg}$ 和 $k_B = W_B/W_\text{avg}$ 保证直线行驶时不会侧向跑偏。解耦前馈
$n_\text{comp} = n_\text{leg}\,(1 + \tfrac{r}{R}\cos\theta)$ 抵消轮毂转动带来的车轮位移，
所以改变底盘高度时车子不会往前一窜。

### COMFORT — 悬挂、调平与 Warp

- **变阻抗**：$K_p = r^2\sin^2\theta\,k_v$ 和 $K_d = r^2\sin^2\theta\,c_v$ 把接地点的虚拟弹簧阻尼
  映射到 HT8115 的 MIT 增益。关节刚度在奇异点处自然趋零。
- **重力前馈** 来自在线质量估计（各腿电流低通滤波）。
- **调平**：Roll / Pitch PID 输出每条腿的高度偏移。
- **Warp**：对角电流差
  $e_\text{warp} = \tfrac12(I_{FL}+I_{BR}) - \tfrac12(I_{FR}+I_{BL})$ 在 COMFORT 下调节刚度，
  在 CLIMBING 下驱动一个 PI 高度修正。

<p align="center">
  <img src="docs/figures/four_dof_modes.png" width="380" alt="Heave / Pitch / Roll / Warp">
</p>

### CLIMBING — 力矩残差检测与运动学攀爬

<p align="center">
  <img src="docs/figures/torque_residual_peaks.png" width="460" alt="500 Hz 下的力矩残差峰值">
</p>

每条腿按 **IDLE → PREP → DETECT → CLIMBING → COMPLETE** 运行。前轮对先爬，两条前腿都完成后，
后轮对自动开始。

- **检测**：残差 $\tau_\text{res} = \tau_\text{fb} - \tau_\text{gravity}$ 与自适应低通基线比较，
  偏差超过 3.0 N·m 并持续 20 ms 即确认接触到台阶。因为判据是"相对变化"，载重和倾斜带来的漂移不会误触发。
- **轨迹**：车轮翻越高度为 *h* 的台阶边缘时满足约束
  $\cos\theta = (R + r - h - R\sin\beta)/r$，由此得到随翻转角 β 变化的腿角度。
- **俯仰偏置**：前腿爬升时车身后仰，后腿爬升时前倾，把重量压到作为支撑的轮子上。

完整推导见 [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md)
（[中文版](docs/TECHNICAL_REPORT_zh.md)）。

---

## 目录结构

```
Circle_Leg_V1/
├── Applications/                   # 应用层
│   ├── Chassis.hpp/.cpp            # 模式状态机、调平、逆运动学、攀爬调度
│   ├── Chassis_Task.hpp/.cpp       # FreeRTOS 500 Hz 控制任务
│   ├── Wheel_Leg.hpp/.cpp          # 单个角：M3508 速度 PID + HT8115 MIT 腿指令链
│   ├── Impedance_Controller.*      # 变阻抗悬挂
│   ├── Ground_Contact.*            # Warp PI 补偿器
│   ├── Climbing_Dynamics.*         # 逐腿攀爬状态机、力矩残差检测
│   ├── Controller.*                # 摇杆 → (Vx, Wz)
│   ├── PC_Comm.*                   # 上位机串口链路、断连看门狗
│   ├── Robot_Config.* / Robot_Params.hpp   # 电机、ID、几何、增益
│   └── Comm_Msg.hpp, Cust_Types.hpp, Helper.hpp
├── Core/                           # CubeMX 外设初始化 + UserTask.cpp
├── Drivers/, Middlewares/          # STM32G4 HAL、CMSIS、CMSIS-DSP（第三方）
├── RM2025-Core/                    # 子模块 → 私有 RM2025-Core（tag circle-leg-v1）
├── docs/
│   ├── TECHNICAL_REPORT.md / TECHNICAL_REPORT_zh.md
│   ├── CODE_STRUCTURE.txt
│   ├── control-chains/             # COMFORT / CLIMBING 控制链路图（HTML）
│   └── figures/
├── Makefile, Core.mk, RM2024-Template-G473.ioc, stm32g473vetx_flash.ld, startup_stm32g473xx.s
└── LICENSE
```

`dev` 分支上有 Jaccob 未合并的实验代码（一个 `WheelModule_t` 电机 / PID 封装）。

---

## 快速开始

需要 `arm-none-eabi-gcc`（用 GCC 12.2 验证过）、GNU Make、SWD 调试器，以及私有仓库
[QuadCirc-Reachable/RM2025-Core](https://github.com/QuadCirc-Reachable/RM2025-Core) 的读取权限
（香港科技大学 ENTERPRIZE RoboMaster 战队的内部库，以子模块方式引用，代码不包含在本仓库内）。

```bash
git clone --recurse-submodules https://github.com/QuadCirc-Reachable/Circle_Leg_V1.0.git
cd Circle_Leg_V1.0
make -j8          # → build/RM2024-Template-G473.elf / .hex / .bin
```

用 Ozone / J-Flash / STM32CubeProgrammer 通过 SWD 烧录 ELF。每个提交都锁定了对应的 RM2025-Core 快照，
所以检出旧提交后执行 `git submodule update` 仍可编译。

---

## 操作说明

| 输入 | 功能 |
|------|------|
| 左摇杆 / 右摇杆 | 前进后退速度 / 转向角速度 |
| `ML` / `MR` | 上一个 / 下一个模式（IDLE → ENERGY_SAVING → COMFORT → CLIMBING → FREE_CONTROL → DEBUG） |
| `ML` + `MR` | CALIBRATION：重新进入 HT8115 电机模式并将腿归零 |
| `A` / `X` / `Y` / `B` | 高度预设 45° / 90° / 145° / 165°（腿向外弯） |
| `LB` / `RB` | 高度预设 90° / 135°（腿向内弯） |
| 扳机（FREE_CONTROL 下） | 直接指定腿角度 |

上位机：[Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0)。V1 使用
**13 字节** 的 `PC_Msg`，没有 D-pad 字节，对应上位机 `7adb072` 及之前的提交；之后的上位机发送的是
14 字节的 V2 格式。

---

## 文档

| 文档 | 内容 |
|------|------|
| [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) | 运动学、状态机、调平、逆运动学、解耦、VMC、阻抗、Warp、攀爬、协议、RTOS |
| [docs/TECHNICAL_REPORT_zh.md](docs/TECHNICAL_REPORT_zh.md) | 中文版技术报告 |
| [docs/CODE_STRUCTURE.txt](docs/CODE_STRUCTURE.txt) | 类层次、各模式控制流、参数 |
| [docs/control-chains/](docs/control-chains/) | COMFORT / CLIMBING 控制链路图 |

---

## 团队

<p align="center">
  <img src="docs/figures/team.jpg" width="560" alt="REACHABLE 团队在港科大 ISD Class of 2026 活动上">
</p>

REACHABLE（QuadCirc），香港科技大学毕业设计项目 SL05a-25。固件由 **LIU Hualin**（嵌入式控制负责人）
开发。团队成员：FANG Ruoyun（感知与人机交互）、WU Ziyao（机械架构与通信）、XU Jusen（机械负责人）。
指导老师：Prof. Chi-Ying TSUI、Prof. Winnie Suk Wai LEUNG；协同指导：Prof. SHI Ling。

## 许可证

应用层代码：[MIT](LICENSE)。`Drivers/`、`Middlewares/` 和模板文件保留各自的许可证。`RM2025-Core`
子模块为私有仓库，不在本许可证覆盖范围内。

## 致谢

香港科技大学 ENTERPRIZE RoboMaster 战队（RM2025-Core、G4 模板）、Jason GAN（HT8115 驱动原作者）、
STMicroelectronics 与 FreeRTOS 项目，以及港科大 ISD 的朋友和同学们。

<p align="center">
  <img src="docs/figures/isd_friends.jpg" width="480" alt="和港科大 ISD 的朋友们合影">
</p>
<p align="center"><sub>和 ISD 的朋友们合影</sub></p>
