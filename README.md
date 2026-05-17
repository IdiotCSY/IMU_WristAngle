# IMU_WristAngle

基于双 IMU 的手腕相对姿态测量实验项目。

本项目使用两个 WT-IMU63 模块分别安装在前臂端和手部端，通过串口读取 IMU 数据，在 MATLAB 中实时计算 hand IMU 相对于 arm IMU 的姿态变化，并进行三维姿态和相对角度可视化。

当前项目包括三部分：

1. `experiment/`：真实 WT-IMU63 双 IMU 实物实验代码；
2. `simulation/`：Simulink 双 IMU 手腕相对姿态仿真；
3. `model/`：用于安装 IMU 和球关节实验的打印件模型。

---

## 1. Project Goal

项目目标是实现双 IMU 手腕相对角度测量：

```text
hand IMU + arm IMU
→ 串口读取 WT-IMU63 数据
→ 姿态解算
→ q_rel = q_arm^{-1} ⊗ q_hand
→ 相对欧拉角 / 轴角计算
→ MATLAB 实时可视化
```

主要关注点包括：

- 动态运动下的实时跟随能力；
- 长时间测量下的姿态漂移；
- 模块内置姿态算法、纯陀螺积分 GI、自适应 Mahony / MH 方法之间的差异；
- 双 IMU 相对姿态在同刚体测试、回零测试和球关节测试中的稳定性。

---

## 2. Repository Structure

```text
IMU_WristAngle/
├── experiment/
│   ├── LIBRARY/          # 串口解析、四元数运算、GI、MH、绘图函数
│   ├── test/             # 测试脚本
│   ├── GI_main.m         # 双 IMU 纯陀螺积分主脚本
│   └── MH_main.m         # 双 IMU 自适应 Mahony 主脚本
│
├── simulation/
│   ├── LIBRARY/          # 仿真工具函数
│   ├── SIMULINK/         # Simulink 模型
│   ├── motion_case/      # 预设运动案例
│   ├── init_wrist_imu_sim.m
│   └── post_wrist_imu_sim.m
│
├── model/                # SolidWorks / STL 打印件模型
├── 调研资料/              # IMU 文档与相关资料
├── .gitignore
└── README.md
```

---

## 3. Hardware Setup

当前实物实验使用：

- IMU module: WT-IMU63
- Software: MATLAB
- Communication: UART serial
- `COM5`: hand IMU
- `COM6`: arm IMU

推荐上位机设置：

### GI / Adaptive MH Mode

```text
输出内容：角速度 + 加速度
通讯速率：115200
回传速率：200 Hz
带宽：188 Hz 或 98 Hz
```

### Built-in Quaternion Test Mode

```text
输出内容：四元数
通讯速率：115200
回传速率：50 Hz 或 200 Hz
```

---

## 4. Implemented Functions

### 4.1 WT-IMU63 Protocol Parser

已完成 WT-IMU63 串口数据帧解析，包括：

```text
0x51: acceleration
0x52: angular velocity
0x53: Euler angle
0x59: quaternion
```

相关函数位于：

```text
experiment/LIBRARY/
```

主要包括：

```matlab
wit_parse_packet.m
wit_parse_stream.m
wit_read_gyro_acc.m
```

---

### 4.2 Quaternion Utilities

四元数工具函数包括：

```matlab
quat_mul.m
quat_conj.m
quat_normalize.m
quat_to_eulZYX_deg.m
quat_to_axis_angle_deg.m
```

相对姿态定义为：

```matlab
qRel = quat_mul(quat_conj(qArm), qHand);
```

即：

```text
q_rel = q_arm^{-1} ⊗ q_hand
```

---

### 4.3 Gyro Integration, GI

GI 方法直接使用角速度进行四元数积分：

```text
q_{k+1} = q_k ⊗ Δq(ω_k)
```

特点：

- 动态响应快；
- 无明显模块内置滤波滞后；
- 长时间运行会存在积分漂移；
- 静止时若角速度输出为 0，则不会继续漂移。

主脚本：

```matlab
experiment/GI_main.m
```

---

### 4.4 Adaptive Mahony / MH

自适应 MH 在 GI 基础上加入加速度计修正：

```text
动态时：主要相信 gyro integration
低动态 / 接近静止时：使用 accelerometer 修正 roll / pitch
```

核心逻辑：

```text
如果 |norm(acc) - g| 较小，并且 norm(gyro) 较小：
    开启加速度计修正
否则：
    降低或关闭加速度计修正
```

注意：

- MH 可以改善 roll / pitch 方向的长期漂移；
- 六轴 IMU 缺少绝对航向参考，因此 MH 不能从根本上解决 yaw 长期漂移；
- 当前版本中 `Ki` 默认关闭，避免引入额外慢漂。

主脚本：

```matlab
experiment/MH_main.m
```

---

## 5. Realtime Visualization

实时显示模块位于：

```matlab
experiment/LIBRARY/imu_realtime_plot_init.m
experiment/LIBRARY/imu_realtime_plot_update.m
experiment/LIBRARY/imu_realtime_plot_reset.m
```

显示内容包括：

- hand IMU 3D attitude；
- arm IMU 3D attitude；
- relative Euler angles: roll / pitch / yaw；
- relative axis-angle rotation angle: theta。

按键：

```text
z: reset zero pose / clear realtime curves
```

---

## 6. Current Experimental Observations

当前阶段的初步实验现象：

1. 模块内置四元数在低速运动下表现较好，但快速动态运动后容易出现滞后和 yaw 漂移。
2. 直接使用角速度做 GI，动态跟随能力明显更好，停下后不会出现模块内置四元数那种明显慢修正。
3. 纯 GI 在长时间动态运动后会有积分累计误差。
4. 自适应 MH 通过在低动态阶段加入加速度计修正，可以降低部分长期漂移。
5. 在同刚体动态测试中，自适应 MH 的残余角度明显低于未修正的 GI 结果。

---

## 7. How to Run

### 7.1 Experiment

进入 `experiment/` 目录：

```matlab
cd experiment
addpath('LIBRARY')
```

运行 GI：

```matlab
GI_main
```

运行自适应 MH：

```matlab
MH_main
```

### 7.2 Simulation

进入 `simulation/` 目录：

```matlab
cd simulation
addpath('LIBRARY')
```

初始化仿真参数：

```matlab
init_wrist_imu_sim
```

运行 Simulink 模型后处理：

```matlab
post_wrist_imu_sim
```

---

## 8. Simulation

`simulation/` 中包含双 IMU 手腕相对姿态仿真框架，主要模块包括：

- 预定义手腕运动；
- IMU 测量模拟；
- 姿态解算；
- 相对姿态计算；
- 误差分析与后处理。

仿真部分用于在实物实验前验证算法逻辑和误差分析流程。

---

## 9. CAD / 3D Printed Parts

打印件模型位于：

```text
model/
```

包含 SolidWorks 零件文件和 STL 文件，用于安装 arm / hand 两侧 IMU 以及后续球关节实验。

---

## 10. Next Steps

后续计划：

- 完善自适应 Mahony 参数调节；
- 增加数据保存和离线后处理；
- 设计同刚体动态零误差测试；
- 设计回零漂移测试；
- 设计已知角度验证实验；
- 使用球关节装置进行三自由度动态测角实验；
- 进一步研究软件约束方法，例如回零约束、关节运动约束等。

---

## 11. Notes

当前项目仍处于实验验证阶段，实时显示结果主要用于观察算法趋势。严格性能评价应基于保存数据后的离线分析，包括：

- final theta；
- max theta；
- RMS theta；
- relative Euler angle drift；
- zero-return error；
- dynamic response delay。