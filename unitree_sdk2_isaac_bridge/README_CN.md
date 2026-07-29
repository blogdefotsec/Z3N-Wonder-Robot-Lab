# Unitree SDK2 Bridge (Isaac Lab / Isaac Sim)

![alt text](doc/image1.png)

把 Unitree SDK2 (DDS) 的低层话题桥接到 Isaac Lab 的机器人仿真中，实现用 `rt/lowcmd` 控制仿真机器人，并发布 `rt/lowstate` 等状态话题。

- 参考实现[unitree_sdk2py_bridge.py](https://github.com/unitreerobotics/unitree_mujoco/blob/main/simulate_python/unitree_sdk2py_bridge.py)

# 依赖

- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab)

## 话题与数据流

**订阅**
- `rt/lowcmd`

**发布**
- `rt/lowstate`
- `rt/sportmodestate`
- `rt/wirelesscontroller`

**控制说明**
- Unitree `LowCmd` 本身包含 `q/dq/kp/kd/tau` 五元组。

## 快速启动

### 1) Isaac Sim / Isaac Lab 启动桥接

以 H2 为例：

```bash
python /media/unitree/HDDStorage/brigham/IsaacLab/scripts/demos/unitree_sdk2_bridge.py \
  --robot h2 \
  --interface lo \
  --physics-dt 0.001 \
  --support-preset full_hoist \
  --startup-hold-mode asset
```

说明：
- 若不传 `--support-preset` / `--support-constraint` / `--enable-hoist` / `--enable-virtual-hand`，默认不启用任何“吊架/支撑/虚拟手”功能，只做 DDS ↔ 仿真关节桥接。
- `--support-preset full_hoist` 当前等价于“给 pelvis 加一个 `prismatic_z` 世界约束”，用于防摔与抬高高度；对应窗口为 `Support Prismatic Z`。

### 2) 外部控制脚本

- domain_id=0（默认）
- 直接发出DDS `LowCmd` 话题到 `rt/lowcmd`，即可控制仿真机器人。

## 支撑/吊架/虚拟手（推荐用法）

这三者目标不同：
- **Support Constraint（推荐）**：用 Physics Joint 把机器人某个刚体 link 约束到世界坐标系。类似于真机的吊架。
- **Hoist（实验）**：外力/硬约束模拟“绳子”。
- **Virtual Hand（实验）**：对某个 body 施加外力，用于抬起/扶住某个部位做交互测试。

### Support Constraint

![alt text](doc/image2.png)

```bash
python /media/unitree/HDDStorage/brigham/IsaacLab/scripts/demos/unitree_sdk2_bridge.py \
  --robot h2 \
  --interface lo \
  --support-constraint prismatic_z \
  --support-constraint-body pelvis
```

GUI/热键（窗口：`Support Prismatic Z`）：
- `Target Z`：支撑目标高度（调这个就会升/降整机）
- `Enable support`：开/关支撑
- `Y`：开/关支撑
- `T/G`：连续升/降 `Target Z`
- `B/N`：步进升/降 `Target Z`

### Virtual Hand（实验）

![alt text](doc/image3.png)

启动时打开工具（随后可用热键开关施力）：

```bash
python /media/unitree/HDDStorage/brigham/IsaacLab/scripts/demos/unitree_sdk2_bridge.py \
  --robot h2 \
  --interface lo \
  --support-preset full_hoist \
  --enable-virtual-hand
```

GUI/热键（窗口：`Virtual Hand Control`）：
- `M`：开/关虚拟手施力
- `P`：把目标点吸附到当前 body 位置
- `J/L`：世界 X 方向移动目标点
- `I/K`：世界 Y 方向移动目标点
- `U/O`：世界 Z 方向移动目标点
- `</>`：切换目标 body

### Hoist（实验，不推荐作为长期支撑）

仅在你明确想测试“外力吊挂”时使用：

```bash
python /media/unitree/HDDStorage/brigham/IsaacLab/scripts/demos/unitree_sdk2_bridge.py \
  --robot h2 \
  --interface lo \
  --enable-hoist \
  --hoist-model elastic
```

说明：
- `--hoist-model hard` 会强行移动 root pose，容易出现“不符合力学”的现象，仅建议调试时短暂使用。
- `wrist_test` 预设会自动启用 `elastic` hoist（轻预载），但稳定性与真实感仍依赖具体场景与参数。

## 关键参数说明

### DDS 与仿真
- `--domain-id`：Cyclone DDS domain id。
- `--interface`：DDS 网卡名。调试建议 `lo`，避免真机/同事进程串台。
- `--physics-dt`：仿真物理步长。
- `--device`：`cpu` 或 `cuda:*`。显存紧张/多人共用时建议 `--device cpu`。

### kp/kd 透传模式（重要）

- `--use-lowcmd-kp-kd`
  - **默认不加**：忽略 lowcmd 的 `kp/kd`；仅使用 `q/dq/tau`：
    - `q` → position target
    - `dq` → velocity target
    - `tau` → effort target（前馈力矩）
    - `kp/kd` → 由仿真侧 implicit actuator 的 stiffness/damping 提供
  - **加上该参数**：使用 lowcmd 的 `kp/kd`，bridge 内显式计算：
    - `tau + kp*(q_des-q) + kd*(dq_des-dq)`

该开关用于在“仿真稳定性”和“复现真机 lowcmd 行为”之间切换。

### 启动保持（站立锁定）

![alt text](doc/image4.png)

- `--hold-default-pose / --no-hold-default-pose`：是否在未收到 lowcmd 前保持姿态。
- `--startup-hold-mode`
  - `asset`：使用资产自带 `default_joint_stiffness/damping` 做轻量保持
  - `lock_current`：锁定启动瞬间姿态（使用下面的 kp/kd）
- `--startup-hold-kp / --startup-hold-kd / --startup-hold-max-tau`：启动保持参数。

### 速度来源
- `--velocity-source sim|fd`
  - `sim`：使用 PhysX 提供的关节速度 `joint_vel`
  - `fd`：使用有限差分 `(q[t]-q[t-1])/dt` 估计速度
- `--velocity-fd-lpf-alpha`：fd 速度的一阶 IIR 平滑（0 禁用）
- `--velocity-fd-max-abs`：fd 速度限幅（0 禁用）

说明：
- 在 `--use-lowcmd-kp-kd` 启用时，速度来源会直接影响 D 项稳定性。
- 在默认（过滤 kp/kd）模式下，速度来源主要用于诊断/启动保持。

### 支撑/吊架/虚拟手
- `--support-preset`：`none|light_support|full_hoist|wrist_test`
- `--support-constraint`：`none|fixed|prismatic_z`（对 pelvis 等 body 做世界约束）
- `--enable-hoist`：Hoist 工具（实验，默认关闭）
- `--enable-virtual-hand`：Virtual Hand 工具（实验，默认关闭）

## 调试与诊断

### 打印低层控制诊断

```bash
--debug-lowcmd-interval-steps 200
--debug-lowcmd-topk 8
--debug-lowcmd-joints "21"
```

诊断输出会包含：
- `lowcmd_rate`、`lowcmd_age`
- `max|q_err|`、`max|dq_err|`
- `max|dq_sim|`（PhysX 原始速度）、`max|dq_used|`（控制使用速度）
- top-k 最大力矩关节列表

### 串台检查（强烈推荐）

bridge 会把 `--mode-machine` 写入 `LowState.mode_machine`。如果外部脚本读到的 `mode_machine` 不是期望值，说明 DDS 话题来源不对（读到了真机或其他进程）。

## 常见问题

### 1) 一启动就爆显存 / PhysX scene 创建失败

多人共用 GPU 或显存紧张时可能出现：
- `CUDA error: out of memory`
- `Unable to allocate ... mGpuContactPairsDev`

建议：
- 使用 CPU 物理：`--device cpu`
- 或使用无渲染：`--headless`

### 2) 一收到 lowcmd 就疯狂抽搐

常见根因：
- `kp/kd`（尤其 D 项）对速度噪声/异常值过敏
- `dq` 来源不可靠（例如 `dq` 被钳位到固定上限或出现尖峰）

建议排查顺序：
- 先用默认模式（不加 `--use-lowcmd-kp-kd`）验证是否稳定
- 若必须复现真机 lowcmd：打开 `--use-lowcmd-kp-kd`，同时使用 `--debug-lowcmd-interval-steps` 观察 `dq_sim/dq_used`

## 版本与约束

- 本脚本以“桥接真机同款 DDS 话题”为目标，但仿真执行器、接触、速度估计链路与真机不同，强 kp/kd 在仿真中更容易触发数值不稳定。
- 推荐先用“过滤 kp/kd”模式建立稳定基线，再逐步启用 Unitree 风格的 kp/kd 透传与更真实的电机/限幅模型。
