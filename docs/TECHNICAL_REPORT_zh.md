# Circle Leg V1 — 毕业设计技术报告（中文版）

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [硬件配置](#3-硬件配置)
4. [偏心轮腿运动学](#4-偏心轮腿运动学)
5. [底盘状态机](#5-底盘状态机)
6. [车身平衡 — Pitch/Roll PID](#6-车身平衡--pitchroll-pid)
7. [逆运动学 — 滑移转向模型](#7-逆运动学--滑移转向模型)
8. [轮腿解耦补偿](#8-轮腿解耦补偿)
9. [虚拟力模型控制 (VMC)](#9-虚拟力模型控制-vmc)
10. [阻抗控制器 — 主动悬挂](#10-阻抗控制器--主动悬挂)
11. [接地补偿 — Warp 模式](#11-接地补偿--warp-模式)
12. [攀爬动力学 — 台阶攀爬](#12-攀爬动力学--台阶攀爬)
13. [电机控制管线](#13-电机控制管线)
14. [通信协议](#14-通信协议)
15. [RTOS 任务结构](#15-rtos-任务结构)
16. [核心公式汇总](#16-核心公式汇总)
17. [文件索引](#17-文件索引)

---

## 1. 项目概述

本项目在 **STM32G473** 微控制器上实现了一个 **四轮偏心轮腿机器人**（Circle Leg），基于 **FreeRTOS** 运行，控制频率为 **500 Hz**。机器人具备以下核心功能：

- **四个独立轮腿单元**，每个由一个 **轮毂电机**（DJI M3508）和一个 **腿部电机**（HT8115，MIT 控制模式）组成。
- **可变阻抗主动悬挂**（舒适模式）：适应不同地形。
- **运动学台阶攀爬**（攀爬模式）：基于力矩残差检测台阶接触，几何轨迹规划补偿腿部角度。
- **接地扭曲补偿**（Warp 补偿）：保证四轮全部着地。
- **PC 遥控**：通过 UART 接收手柄摇杆和按键指令。

---

## 2. 系统架构

```
┌──────────────────────────────────────────────────────┐
│                    PC 控制器                          │
│              (摇杆 + 按键，UART 通信)                  │
└──────────────┬───────────────────────────────────────┘
               │  PC_Msg (13 字节)
               ▼
┌──────────────────────────────────────────────────────┐
│              Chassis_Task (500 Hz 主循环)              │
│  ┌────────────────────────────────────────────────┐  │
│  │          Chassis（底盘状态机）                    │  │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────────┐   │  │
│  │  │ Pitch/   │ │  阻抗    │ │  攀爬动力学   │   │  │
│  │  │ Roll PID │ │  控制器  │ │              │   │  │
│  │  └──────────┘ └──────────┘ └──────────────┘   │  │
│  │  ┌──────────┐ ┌──────────┐                    │  │
│  │  │  接地    │ │  逆运动  │                    │  │
│  │  │  补偿    │ │  学解算  │                    │  │
│  │  └──────────┘ └──────────┘                    │  │
│  └────────────────────────────────────────────────┘  │
│          │            │           │          │        │
│      ┌───┴───┐   ┌───┴───┐  ┌───┴───┐  ┌───┴───┐   │
│      │FL 轮腿│   │FR 轮腿│  │BL 轮腿│  │BR 轮腿│   │
│      │M3508+ │   │M3508+ │  │M3508+ │  │M3508+ │   │
│      │HT8115 │   │HT8115 │  │HT8115 │  │HT8115 │   │
│      └───────┘   └───────┘  └───────┘  └───────┘   │
└──────────────────────────────────────────────────────┘
```

---

## 3. 硬件配置

### 3.1 机械参数

| 参数 | 符号 | 数值 | 单位 |
|------|------|------|------|
| 车轮半径 | $R$ | 160 | mm |
| 偏心距（腿长） | $r$ | 65 | mm |
| 轴距 | $L$ | 270 | mm |
| 前轮距 | $W_F$ | 531 | mm |
| 后轮距 | $W_B$ | 395 | mm |
| 机器人总质量 | $M$ | 10.0 | kg |
| 单腿质量 | $m_{leg}$ | 0.5 | kg |

### 3.2 电机配置

| 电机 | 型号 | 数量 | 总线 | 功能 |
|------|------|------|------|------|
| M3508 | DJI 无刷直流电机 | 4 | CAN 2 | 轮毂驱动，PID 速度控制 |
| HT8115 | 力矩电机 | 4 | CAN 3 | 腿部驱动，MIT 阻抗控制 |

### 3.3 惯性测量单元 (IMU)

- 板载 IMU 提供欧拉角（Yaw、Pitch、Roll）、校准陀螺仪数据及去重力后的地球坐标系线性加速度。
- IMU 安装 Roll 偏移量：**182.91°**（软件补偿）。
- 角度低通滤波器：$\alpha = 0.02$（500 Hz 下截止频率约 4 Hz）。

---

## 4. 偏心轮腿运动学

每个轮腿单元采用 **偏心连杆机构**：腿部电机旋转角度 $\theta$，改变底盘离地高度。

### 4.1 高度–角度关系

$$H(\theta) = R + r \cdot \cos(\theta)$$

其中：
- $H$ = 地面到电机中心的高度（m）
- $R$ = 车轮半径（m）
- $r$ = 偏心距 / 腿长（m）
- $\theta$ = 腿部角度（0° = 完全伸展 / 最高，180° = 完全收缩 / 最低）

### 4.2 逆解：高度转角度

给定目标高度 $H_{target}$：

$$\cos(\theta) = \frac{H_{target} - R}{r}$$

$$\theta = \arccos\left(\frac{H_{target} - R}{r}\right)$$

**安全限制：** 角度钳制在 $[10°, 170°]$，避免 0° 和 180° 处的奇异点。

### 4.3 速度前馈（高度变化率转角速度）

由 $\dot{H} = -r \sin(\theta) \cdot \dot{\theta}$ 得：

$$\dot{\theta} = \frac{-\dot{H}}{r \cdot (\sin(\theta) + \epsilon)}$$

其中 $\epsilon = 0.1$ 为阻尼项，防止奇异点处除零。

### 4.4 弯曲方向

每条腿有一个 `bending_direction` 参数（$+1$ 或 $-1$），选择正角度或负角度解，使机器人可在"外展"和"内收"两种腿部构型之间切换。

---

## 5. 底盘状态机

系统通过手柄按键控制，在以下状态间切换：

| 状态 | 编号 | 说明 |
|------|------|------|
| **CALIBRATION（校准）** | 0 | 重新进入 HT 电机模式，设置零位，然后转入 IDLE |
| **IDLE（待机）** | 1 | 所有电机停止，零输出 |
| **ENERGY_SAVING（省电）** | 2 | 腿部折叠至 0°，刚性位置保持（$K_p=20$, $K_d=1.5$），可基本驱动 |
| **COMFORT（舒适）** | 3 | **可变阻抗主动悬挂**：IMU 车身平衡 + Skyhook 阻尼 + Warp 补偿 |
| **CLIMBING（攀爬）** | 4 | **逐腿运动学台阶攀爬**：力矩残差检测 + 前腿先行后腿跟进 |
| **FREE_CONTROL（自由控制）** | 5 | 扳机直接映射腿部角度，手动操控 |
| **DEBUG（调试）** | 6 | 按键控制高度，不启用 PID 平衡（用于测试） |

### 模式切换

- **ML + MR 同时按下** → 进入 CALIBRATION
- **ML**（上升沿）→ 切换到上一个状态
- **MR**（上升沿）→ 切换到下一个状态

### 退出 COMFORT 模式的平滑 Kp 过渡

离开 COMFORT 模式时，捕获当前阻抗 $K_p$/$K_d$ 值，在 **200 帧（0.4 秒）** 内线性过渡到目标模式的参数，防止力矩突变：

$$K_p(t) = K_{p,\text{exit}} + \alpha(t) \cdot (K_{p,\text{target}} - K_{p,\text{exit}})$$

其中 $\alpha(t) = t / T_{ramp}$，从 0 递增到 1。

---

## 6. 车身平衡 — Pitch/Roll PID

两个独立 PID 控制器根据 IMU 反馈维持底盘水平：

| PID | $K_p$ | $K_i$ | $K_d$ | 积分限幅 | 输出限幅 |
|-----|--------|--------|--------|----------|----------|
| Roll（横滚） | 0.015 | 0.0002 | 0.00012 | 1000 | 0.08 m |
| Pitch（俯仰） | 0.015 | 0.00015 | 0.00012 | 1000 | 0.08 m |

**PID 输出**为高度偏移量（单位：米）。四腿分配规则：

| 腿 | Pitch 符号 | Roll 符号 |
|----|------------|-----------|
| FL（前左） | $+1$ | $-1$ |
| FR（前右） | $+1$ | $+1$ |
| BL（后左） | $-1$ | $-1$ |
| BR（后右） | $-1$ | $+1$ |

每条腿的目标高度：

$$H_i = H_{target} + s_{pitch,i} \cdot \Delta H_{pitch} + s_{roll,i} \cdot \Delta H_{roll} + \Delta H_{mode,i}$$

其中：
- $\Delta H_{pitch} = \text{PID}_{pitch}(0, \theta_{pitch})$
- $\Delta H_{roll} = \text{PID}_{roll}(0, \theta_{roll})$
- $\Delta H_{mode,i}$ = 模式特定补偿量（Warp、攀爬等）

---

## 7. 逆运动学 — 滑移转向模型

机器人采用 **非对称滑移转向**（$W_F \neq W_B$）：

$$V_{FL} = V_x - \omega_z \cdot k_F, \qquad V_{FR} = V_x + \omega_z \cdot k_F$$

$$V_{BL} = V_x - \omega_z \cdot k_B, \qquad V_{BR} = V_x + \omega_z \cdot k_B$$

其中每个轴的差速增益为：

$$k_F = \frac{W_F}{W_{avg}}, \qquad k_B = \frac{W_B}{W_{avg}}, \qquad W_{avg} = \frac{W_F + W_B}{2}$$

**归一化：** 若任一轮 RPM 超过 `MAX_WHEEL_RPM`（80 RPM），则等比例缩小全部四轮 RPM。

### 摇杆映射

- **左摇杆** → 前进/后退速度 $V_x$（最大 ±40 RPM）
- **右摇杆** → 旋转速度 $\omega_z$（最大 ±40 RPM）
- 死区：半径幅值 200/1000

---

## 8. 轮腿解耦补偿

当腿部电机旋转时，会通过机械结构耦合到车轮，导致非预期的轮速变化。补偿公式：

$$n_{wheel,comp} = n_{leg} \cdot \left(1 + \frac{r}{R} \cdot \cos(\theta_{leg})\right) \cdot k_{coupling}$$

其中：
- $n_{leg}$ = 腿部电机转速
- $r / R$ = 偏心距 / 车轮半径
- $\theta_{leg}$ = 当前腿部角度
- $k_{coupling}$ = 耦合方向符号（$\pm 1$，取决于安装方式）

该补偿量在 PID 执行前加到车轮电机目标转速上。

---

## 9. 虚拟力模型控制 (VMC)

通过机构雅可比矩阵将期望垂直力 $F_z$ 转换为电机力矩：

$$\tau = F_z \cdot r \cdot \cos(\theta)$$

其中 $\tau$ 为电机力矩（Nm），$r$ 为偏心距（m），$\theta$ 为腿部角度（rad）。

### 重力补偿

每条腿持续补偿重力力矩：

$$\tau_{gravity} = -(m_{leg} \cdot g \cdot r \cdot \cos(\theta))$$

以 **前馈力矩** 的形式在 MIT 控制指令中持续施加。

---

## 10. 阻抗控制器 — 主动悬挂

作为 COMFORT 模式的核心算法，将期望的 **笛卡尔空间阻抗**（轮地接触点处的弹簧-阻尼器）映射为 **关节空间 MIT 电机参数**。

### 10.1 理论推导

**偏心连杆雅可比矩阵：**

$$J(\theta) = \frac{dH}{d\theta} = -r \cdot \sin(\theta)$$

**期望笛卡尔阻抗：**

$$F = k_v \cdot (H_{des} - H) + c_v \cdot (\dot{H}_{des} - \dot{H}) + F_{gravity}$$

**映射到 MIT 控制律**（$\tau = K_p \cdot \delta\theta + K_d \cdot \delta\dot{\theta} + \tau_{ff}$）：

$$\boxed{K_{p,MIT} = r^2 \cdot \sin^2(\theta) \cdot k_v}$$

$$\boxed{K_{d,MIT} = r^2 \cdot \sin^2(\theta) \cdot c_v}$$

$$\boxed{\tau_{ff} = -F_{load} \cdot r \cdot \sin(\theta)}$$

**数值示例**（$\theta=90°$, $r=0.065$ m）：
- $r^2 \sin^2(90°) = 0.004225$
- $k_v = 800$ N/m → $K_p = 3.38$ Nm/rad
- $c_v = 250$ Ns/m → $K_d = 1.06$ Nms/rad

### 10.2 天棚阻尼（Kd 调制）

通过刚体运动学计算每条腿的垂直速度：

$$V_{FL} = V_z - \omega_r \cdot \frac{W_F}{2} + \omega_p \cdot \frac{L}{2}$$

$$V_{FR} = V_z + \omega_r \cdot \frac{W_F}{2} + \omega_p \cdot \frac{L}{2}$$

$$V_{BL} = V_z - \omega_r \cdot \frac{W_B}{2} - \omega_p \cdot \frac{L}{2}$$

$$V_{BR} = V_z + \omega_r \cdot \frac{W_B}{2} - \omega_p \cdot \frac{L}{2}$$

天棚阻尼定律：

$$c_{v,i} = c_{base} + c_{sky} \cdot |V_{leg,i}|$$

> 注：目前已禁用（$c_{sky} = 0$），原因是加速度计噪声导致速度估计不可靠。

### 10.3 Warp Kp 调制（接地补偿）

对角载荷不平衡检测：

$$e_{warp} = \frac{I_{FL} + I_{BR}}{2} - \frac{I_{FR} + I_{BL}}{2}$$

Warp 符号模式：$\{FL: +1, FR: -1, BL: -1, BR: +1\}$

$$k_{v,i} = k_{v,base} \cdot \left(1 - \gamma \cdot s_{warp,i} \cdot \text{clamp}(e_{warp})\right)$$

**效果：** 过载对角线变软 → 压缩 → 将载荷重新分配到欠载对角线。

### 10.4 动态载荷估计（前馈力矩）

通过电机电流估计总质量（慢速低通滤波）：

$$M_{est} = \frac{\text{LPF}(\sum |I_i|) \cdot K_A \cdot GR}{g}$$

每条腿的重力补偿前馈：

$$\tau_{ff,i} = -\left(\frac{M_{est}}{4} + m_{leg}\right) \cdot g \cdot r \cdot \sin(\theta_i) + \tau_{warp,i}$$

### 10.5 模式进入过渡

进入 COMFORT 模式后，$K_p$/$K_d$ 从初始默认值（$K_p=35$, $K_d=1.5$）渐变到阻抗计算值，约 0.4 秒过渡：

$$K_p(t) = K_{p,entry} + \alpha(t) \cdot (K_{p,impedance} - K_{p,entry})$$

其中 $\alpha$ 每周期递增 0.005，直到达到 1.0。

---

## 11. 接地补偿 — Warp 模式

### 11.1 问题描述

刚性底盘在四个接触点上存在 **一个过约束自由度**（对角扭曲）。Pitch 和 Roll PID 控制了 3 个倾斜自由度中的 2 个，剩余的 Warp（扭曲）自由度未被控制 —— 可能导致一个车轮翘起离地。

### 11.2 Warp 正交分解

四条腿的高度偏移分解为四个正交模式：

| 模式 | 分配模式 (FL, FR, BL, BR) | 控制器 |
|------|--------------------------|--------|
| 升降（Heave） | $(+1, +1, +1, +1)$ | `target_chassis_height_` |
| 俯仰（Pitch） | $(+1, +1, -1, -1)$ | Pitch PID |
| 横滚（Roll） | $(-1, +1, -1, +1)$ | Roll PID |
| **扭曲（Warp）** | $(+1, -1, -1, +1)$ | **GroundContact PI 控制器** |

### 11.3 PI 控制器

$$\Delta H_{warp} = K_p \cdot e_{warp} + K_i \cdot \int e_{warp} \, dt$$

| 参数 | 数值 |
|------|------|
| $K_p$ | 0.002 m/A |
| $K_i$ | 0.0005 m/(A·s) |
| 最大 $\Delta H$ | ±20 mm |
| 死区 | 0.15 A |

各腿输出：

$$\Delta H_{FL} = -\Delta H_{warp}, \quad \Delta H_{FR} = +\Delta H_{warp}$$
$$\Delta H_{BL} = +\Delta H_{warp}, \quad \Delta H_{BR} = -\Delta H_{warp}$$

**抗积分饱和：** 积分项限幅并施加衰减（死区内 ×0.990，死区外 ×0.999）。

---

## 12. 攀爬动力学 — 台阶攀爬

### 12.1 逐腿状态机

```
IDLE → PREP → DETECT → CLIMBING → COMPLETE → (返回 IDLE)
```

| 阶段 | 说明 |
|------|------|
| **IDLE** | 正常高度控制管线 |
| **PREP（准备）** | 将腿移至准备角度（$180° - \theta_{prep}$，默认 165°）以避开奇异点 |
| **DETECT（检测）** | 保持准备角度，监测 **腿部力矩残差** 以检测台阶接触 |
| **CLIMBING（攀爬）** | 执行 **运动学轨迹** — 随车轮滚过台阶边缘补偿腿部角度 |
| **COMPLETE（完成）** | 保持攀爬结束角度，等待重置 |

**执行顺序：** 前腿（FL、FR）先攀爬。当两条前腿均到达 COMPLETE 后，后腿（BL、BR）自动开始。

### 12.2 台阶检测 — 力矩残差法

力矩残差隔离台阶接触力：

$$\tau_{residual} = \tau_{feedback} - \tau_{gravity\_comp}$$

慢速低通滤波跟踪直流基线：

$$\tau_{baseline}(k) = \alpha \cdot \tau_{residual}(k) + (1-\alpha) \cdot \tau_{baseline}(k-1)$$

当以下条件持续 $t_{confirm} = 20$ ms 时触发检测：

$$|\tau_{residual} - \tau_{baseline}| > \tau_{threshold}$$

默认阈值：**3.0 Nm**。

**预热阶段：** 前 0.1 秒使用快速 LPF（$\alpha=0.3$）快速收敛基线，之后切换为慢速（$\alpha=0.02$）。

### 12.3 攀爬运动学模型

车轮（半径 $R$）通过偏心腿（长度 $L$）攀爬高度为 $h$ 的台阶。车轮绕台阶边缘点 $E$ 旋转。

**约束方程：**

$$\cos(\theta) = \frac{R + L - h - R \cdot \sin(\beta)}{L}$$

其中：
- $\theta$ = 运动学模型中的腿部角度（小 $\theta$ = 伸展）
- $\beta$ = 车轮绕台阶边缘旋转的角度（$\beta_0$ → $90°$）

**角速度耦合关系：**

$$\dot{\theta} = \frac{R \cdot \cos(\beta) \cdot \dot{\beta}}{L \cdot \sin(\theta)}$$

**边界条件：**
- 起始：$\theta = \theta_{prep}$，$\beta = \beta_0 = \arcsin\left(\frac{R + L(1 - \cos\theta_{prep}) - h}{R}\right)$
- 结束：$\beta = 90°$，$\theta_{end} = \arccos\left(\frac{L - h}{L}\right)$

### 12.4 轨迹生成

攀爬角度 $\beta$ 以 **恒定虚拟角速度** $\omega_{climb}$（默认 1.0 rad/s）积分，不依赖实际车轮转速（车轮在台阶边缘堵转时 $\phi_w \approx 0$）：

$$\beta(k+1) = \beta(k) + \omega_{climb} \cdot \Delta t$$

每步计算：
1. 由约束方程求 $\theta$：$\cos(\theta) = \frac{R + L - h - R \sin(\beta)}{L}$
2. 电机角度：$\theta_{motor} = 180° - \theta$（约定：180° = 最高点）
3. 施加攀爬方向：FL/FR 使用 $-\theta_{motor}$，BL/BR 使用 $+\theta_{motor}$

### 12.5 攀爬时的车轮前进驱动

在 CLIMBING 和 COMPLETE 阶段，叠加前进 RPM 增量：

$$n_{climb} = \omega_{climb} \cdot \frac{60}{2\pi} \cdot k_{scale}$$

其中 $k_{scale}$（默认 5.0）为车轮 RPM 倍增系数。仅当用户推杆前进（$V_x > 0.5$）时激活。

### 12.6 攀爬时的 Pitch 偏置

通过 Pitch 倾斜将重心偏向攀爬轮：
- **前腿攀爬** → 向前倾斜（Pitch 设定值 $-3°$）
- **后腿攀爬** → 向后倾斜（Pitch 设定值 $+3°$）

---

## 13. 电机控制管线

### 13.1 车轮电机管线（M3508）

```
目标 RPM → + 解耦补偿 → 死区判断 → PID → 电机输出
```

**车轮速度 PID** 参数（每个电机相同）：
- $K_p = 200$，$K_i = 180$，$K_d = 1.0$
- 积分限幅：15000，输出限幅：16000

**死区：** 当目标和反馈转速均低于 15 RPM 时，输出置零并重置 PID，防止积分饱和。

### 13.2 腿部电机管线（HT8115 — MIT 模式）

```
目标高度 → 高度转角度 → 斜率限制器 → MIT 指令
```

**MIT 控制律（HT8115）：**

$$\tau = K_p \cdot (\theta_{target} - \theta_{fb}) + K_d \cdot (\dot{\theta}_{target} - \dot{\theta}_{fb}) + I_{ff} \cdot K_A$$

其中：
- $K_p$ = 位置刚度（Nm/rad，范围 0–500）
- $K_d$ = 速度阻尼（Nms/rad，范围 0–5）
- $I_{ff}$ = 前馈电流（A）
- $K_A$ = 电机力矩常数

**默认 MIT 参数：** $K_p = 35$，$K_d = 1.5$（刚性位置保持）。

### 13.3 斜率限制器（Slew Rate Limiter）

防止腿部暴力运动，每周期角度变化钳制：

$$|\Delta\theta_{cmd}| \leq \dot{\theta}_{max} \cdot \Delta t = 400°/\text{s} \times 2\text{ms} = 0.8°/\text{周期}$$

包含 **角度展开**（Unwrap）逻辑：正确处理 ±180° 的角度回绕问题。

---

## 14. 通信协议

### 14.1 PC → MCU（PC_Msg，13 字节）

| 字段 | 类型 | 说明 |
|------|------|------|
| `left_joystick` | `{angle_x10, r_x1000}` | 左摇杆：前进/后退 |
| `right_joystick` | `{angle_x10, r_x1000}` | 右摇杆：旋转 |
| `Left_trigger_x1000` | `uint16` | 左扳机（0–1000） |
| `Right_trigger_x1000` | `uint16` | 右扳机（0–1000） |
| `button_status` | `uint8` | 8 个按键位：LB, RB, X, A, B, Y, ML, MR |

### 14.2 MCU → PC（Reachable_Msg，40 字节）

- 电机当前/目标位置（×10）、温度
- 电机当前/目标转速、温度

### 14.3 连接检测

- 基于超时机制：若超时未收到消息，机器人自动进入 IDLE。
- 重连时按键状态强制置为 0xFF，防止误触发上升沿。

---

## 15. RTOS 任务结构

| 任务 | 栈大小 | 优先级 | 频率 | 功能 |
|------|--------|--------|------|------|
| `Chassis_Task` | 2048 字 | 0 | 500 Hz（2 ms 延时） | 主控制循环 |
| `PC_CommTask` | — | — | 100 ms 发送 | UART 通信 |

### 启动流程

1. 等待 3000 ms（HT8115 电机上电初始化 CAN 接口）
2. 调用 `chassis.Init()` — 使能所有电机，向 HT 电机发送 ENTER_MOTOR ×20
3. 进入主循环：读取 PC 指令 → 更新底盘控制 → 发送 CAN 指令 → 回传反馈

---

## 16. 核心公式汇总

### 运动学

| 公式 | 方程 | 所在函数 |
|------|------|----------|
| 角度转高度 | $H = R + r\cos\theta$ | `Set_Leg_Height`, `CalculateHeightFromAngle` |
| 高度转角度 | $\theta = \arccos\left(\frac{H-R}{r}\right)$ | `Set_Leg_Height` |
| 速度前馈 | $\dot\theta = \frac{-\dot H}{r(\sin\theta + \epsilon)}$ | `Set_Leg_Height` |
| 轮腿解耦 | $n_w = n_l \left(1 + \frac{r}{R}\cos\theta\right)$ | `Wheel_Compensation` |
| VMC（雅可比） | $\tau = F_z \cdot r \cdot \cos\theta$ | `VMC_Calculation` |
| 重力补偿 | $\tau_g = -m \cdot g \cdot r \cdot \cos\theta$ | `Get_LegGravityTorque` |

### 阻抗控制

| 公式 | 方程 |
|------|------|
| MIT Kp 映射 | $K_{p} = r^2 \sin^2\theta \cdot k_v$ |
| MIT Kd 映射 | $K_{d} = r^2 \sin^2\theta \cdot c_v$ |
| 前馈力矩 | $\tau_{ff} = -F_{load} \cdot r \cdot \sin\theta$ |
| Warp 刚度调制 | $k_{v,i} = k_{v,base}(1 - \gamma \cdot s_i \cdot \hat{e}_{warp})$ |
| 质量估计 | $M = \text{LPF}(\sum|I_i|) \cdot K_A \cdot GR / g$ |

### 攀爬

| 公式 | 方程 |
|------|------|
| 约束方程 | $\cos\theta = \frac{R + L - h - R\sin\beta}{L}$ |
| 角速度耦合 | $\dot\theta = \frac{R\cos\beta \cdot \dot\beta}{L\sin\theta}$ |
| 结束角度 | $\theta_{end} = \arccos\left(\frac{L-h}{L}\right)$ |
| 起始角度 | $\beta_0 = \arcsin\left(\frac{R + L(1 - \cos\theta_{prep}) - h}{R}\right)$ |

### 逆运动学（滑移转向）

| 车轮 | 公式 |
|------|------|
| FL | $V_x - \omega_z \cdot k_F$ |
| FR | $V_x + \omega_z \cdot k_F$ |
| BL | $V_x - \omega_z \cdot k_B$ |
| BR | $V_x + \omega_z \cdot k_B$ |

其中 $k_F = W_F / W_{avg}$，$k_B = W_B / W_{avg}$。

---

## 17. 文件索引

| 文件 | 说明 |
|------|------|
| `Chassis.hpp / .cpp` | 底盘状态机、模式处理、车身平衡、逆运动学 |
| `Wheel_Leg.hpp / .cpp` | 单腿电机控制：高度→角度转换、MIT 指令、轮毂 PID、解耦补偿 |
| `Impedance_Controller.hpp / .cpp` | 可变阻抗主动悬挂：笛卡尔→MIT 映射、天棚阻尼、Warp Kp 调制 |
| `Climbing_Dynamics.hpp / .cpp` | 逐腿攀爬状态机、运动学轨迹、力矩残差台阶检测 |
| `Ground_Contact.hpp / .cpp` | Warp 模式 PI 补偿器，保证四轮接地 |
| `Controller.hpp / .cpp` | 摇杆到速度的映射 |
| `Robot_Params.hpp` | 全部机械/控制常量定义 |
| `Robot_Config.cpp` | 电机/PID/轮腿实例化与接线配置 |
| `Chassis_Task.cpp` | FreeRTOS 任务：500 Hz 主控制循环 |
| `PC_Comm.hpp / .cpp` | UART 与 PC 控制器通信 |
| `Comm_Msg.hpp` | 通信协议消息结构体（PC_Msg、Reachable_Msg） |
| `Cust_Types.hpp` | 枚举/结构体定义（Chassis_State、MIT_Params、Wheel_Leg_Params） |
| `Helper.hpp` | 工具函数（角度归一化、角度/弧度转换） |
