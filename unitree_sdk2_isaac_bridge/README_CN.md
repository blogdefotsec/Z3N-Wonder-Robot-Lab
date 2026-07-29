# Unitree SDK2 Bridge (Isaac Lab / Isaac Sim)

![alt text](doc/image1.png)

把 Unitree SDK2 (DDS) 的低层话题桥接到 Isaac Lab 的机器人仿真中，实现用 `rt/lowcmd` 控制仿真机器人，并发布 `rt/lowstate` 等状态话题。

- 参考实现[unitree\_sdk2py\_bridge.py](https://github.com/unitreerobotics/unitree_mujoco/blob/main/simulate_python/unitree_sdk2py_bridge.py)

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
  --h2-usd /path/to/h2/usd \
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

- domain\_id=0（默认）
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
python unitree_sdk2_bridge.py \
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

# 参数一览表

## Bridge 参数（unitree\_sdk2\_bridge.py）

| 参数                                               | 功能                                                                     | 是否必填（不填则默认值）                                                                             |
| ------------------------------------------------ | ---------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `--robot`                                        | 选择要生成的机器人模型                                                            | 否（默认 `h2`）                                                                               |
| `--domain-id`                                    | Cyclone DDS domain id                                                  | 否（默认 `0`）                                                                                |
| `--interface`                                    | DDS 网卡名（传给 `ChannelFactoryInitialize`）                                 | 否（默认空字符串 `""`，通常建议显式传 `lo` 或实际网卡名）                                                       |
| `--physics-dt`                                   | 仿真物理步长                                                                 | 否（默认 `0.002`）                                                                            |
| `--render-interval`                              | 每渲染一帧跨多少个 physics step（越大 GUI 越流畅/物理相对更“快”）                            | 否（默认 `4`）                                                                                |
| `--mode-machine`                                 | 写入 `LowState.mode_machine` 用于串台/来源标记                                   | 否（默认 `7`）                                                                                |
| `--velocity-source`                              | 控制用速度来源：`sim`=PhysX `joint_vel`；`fd`=位置差分                              | 否（默认 `sim`）                                                                              |
| `--velocity-fd-lpf-alpha`                        | `fd` 速度一阶 IIR 平滑系数（0 禁用）                                               | 否（默认 `0.0`）                                                                              |
| `--velocity-fd-max-abs`                          | `fd` 速度限幅（rad/s，0 禁用）                                                  | 否（默认 `0.0`）                                                                              |
| `--physx-enable-external-forces-every-iteration` | 启用 PhysX TGS “每个 position iteration 都施加 external forces”，用于降低速度噪声      | 否（默认关闭 `False`；传了即为开启）                                                                   |
| `--physx-min-velocity-iterations`                | PhysX solver 最小 velocity iteration 数（提升速度准确性）                          | 否（默认 `0`）                                                                                |
| `--support-preset`                               | 支撑预设：`none/light_support/full_hoist/wrist_test`                        | 否（默认 `none`）要启用默认吊架，推荐full\_hoist                                                        |
| `--support-constraint`                           | 世界约束类型：`none/fixed/prismatic_z`                                        | 否（默认 `none`）                                                                             |
| `--support-constraint-body`                      | 支撑约束绑定的刚体 body 名                                                       | 否（默认 `pelvis`）                                                                           |
| `--support-prismatic-lower`                      | prismatic\_z 支撑的下限（m）                                                  | 否（默认 `-10.0`）                                                                            |
| `--support-prismatic-upper`                      | prismatic\_z 支撑的上限（m）                                                  | 否（默认 `10.0`）                                                                             |
| `--support-prismatic-drive-stiffness`            | prismatic\_z 驱动刚度                                                      | 否（默认 `8000.0`）                                                                           |
| `--support-prismatic-drive-damping`              | prismatic\_z 驱动阻尼                                                      | 否（默认 `1200.0`）                                                                           |
| `--support-prismatic-drive-max-force`            | prismatic\_z 驱动最大力                                                     | 否（默认 `200000.0`，即 `2e5`）                                                                 |
| `--hold-default-pose`                            | 启动后在没收到 lowcmd 前维持姿态（轻量保持）                                             | 否（默认开启 `True`；传不传都为 True，这个参数主要用于“显式写全”）                                                 |
| `--startup-hold-mode`                            | 启动保持模式：`asset` 或 `lock_current`                                        | 否（默认 `asset`）                                                                            |
| `--startup-hold-kp`                              | 启动保持 P 增益（部分模式/配置会用到）                                                  | 否（默认 `120.0`）                                                                            |
| `--startup-hold-kd`                              | 启动保持 D 增益（部分模式/配置会用到）                                                  | 否（默认 `6.0`）                                                                              |
| `--startup-hold-max-tau`                         | 启动保持力矩限幅                                                               | 否（默认 `120.0`）                                                                            |
| `--status-interval-steps`                        | 每 N 步打印一次基础状态（0 关闭）                                                    | 否（默认 `1000`）                                                                             |
| `--debug-lowcmd-interval-steps`                  | 每 N 步打印一次 lowcmd 详细诊断（0 关闭）                                            | 否（默认 `0`）                                                                                |
| `--debug-lowcmd-topk`                            | 诊断里打印力矩 top-k 关节数                                                      | 否（默认 `8`）                                                                                |
| `--debug-lowcmd-joints`                          | 诊断里强制打印的 sdk 关节 index（逗号分隔字符串）                                         | 否（默认空字符串 `""`）                                                                           |
| `--h2-usd`                                       | H2 用的 USD 路径                                                           | H2 USD地址。要使用H2，必须传入这个地址。                                                                 |
| `--h2-articulation-root`                         | H2 USD 内 articulation root prim path（相对加载的 USD root，且以 `/` 开头）；留空则自动解析 | 否（默认 `""`）                                                                               |
| `--h2-linear-damping`                            | 给 H2 USD 应用的刚体线阻尼                                                      | 否（默认 `0.2`）                                                                              |
| `--h2-angular-damping`                           | 给 H2 USD 应用的刚体角阻尼                                                      | 否（默认 `0.4`）                                                                              |
| `--disable-hoist`                                | 显式关闭 hoist 工具                                                          | 否（默认 hoist 关闭：`--enable-hoist` 默认 `False`；传 `--disable-hoist` 也是关闭）                      |
| `--hoist-model`                                  | Hoist 模型：`elastic` 或 `hard`                                            | 否（默认 `elastic`）                                                                          |
| `--hoist-body`                                   | Hoist 绑定 body 名                                                        | 否（默认 `torso_link`）                                                                       |
| `--hoist-stiffness`                              | Elastic hoist 刚度                                                       | 否（默认 `220.0`）                                                                            |
| `--hoist-damping`                                | Elastic hoist 阻尼                                                       | 否（默认 `90.0`）                                                                             |
| `--hoist-planar-stiffness`                       | 平面回正刚度（抑制飘移）                                                           | 否（默认 `80.0`）                                                                             |
| `--hoist-planar-damping`                         | 平面回正阻尼（抑制摆动）                                                           | 否（默认 `120.0`）                                                                            |
| `--hoist-rest-length`                            | 绳长（松弛长度）                                                               | 否（默认 `0.0`）                                                                              |
| `--hoist-height-offset`                          | 初始 anchor 高度偏置（m）                                                      | 否（默认 `0.8`）                                                                              |
| `--hoist-height-rate`                            | 按键持续调高度的速度（m/s）                                                        | 否（默认 `0.4`）                                                                              |
| `--hoist-height-step`                            | 单次按键调高度的步进（m）                                                          | 否（默认 `0.05`）                                                                             |
| `--hoist-max-force`                              | Hoist 力限幅                                                              | 否（默认 `450.0`）                                                                            |
| `--hoist-preload-force`                          | Hoist 额外向上预载力                                                          | 否（默认 `0.0`）                                                                              |
| `--hoist-auto-preload-scale`                     | auto preload 的缩放系数（需要配合 `--hoist-auto-preload` 才生效）                    | 否（默认 `1.0`）                                                                              |
| `--hoist-debug-interval`                         | 每 N 步打印 hoist debug（0 关闭）                                              | 否（默认 `0`）                                                                                |
| `--disable-virtual-hand`                         | 显式关闭 virtual hand 工具                                                   | 否（默认 virtual hand 关闭：`--enable-virtual-hand` 默认 `False`；传 `--disable-virtual-hand` 也是关闭） |
| `--hand-body`                                    | virtual hand 优先绑定的 body 名                                              | 否（默认 `right_hand_link`）                                                                  |
| `--hand-stiffness`                               | virtual hand 笛卡尔刚度                                                     | 否（默认 `700.0`）                                                                            |
| `--hand-damping`                                 | virtual hand 笛卡尔阻尼                                                     | 否（默认 `120.0`）                                                                            |
| `--hand-max-force`                               | virtual hand 力限幅                                                       | 否（默认 `1200.0`）                                                                           |
| `--hand-position-step`                           | virtual hand GUI/热键平移步进                                                | 否（默认 `0.03`）                                                                             |

## AppLauncher 参数（Isaac Sim 启动器相关）

| 参数                            | 功能                                                  | 是否必填（不填则默认值）           |
| ----------------------------- | --------------------------------------------------- | ---------------------- |
| `--device`                    | 仿真设备：`cpu` / `cuda` / `cuda:N`                      | 否（默认 `cuda:0`）         |
| `--enable_cameras`            | 启用相机传感器与相关渲染依赖                                      | 否（默认关闭 `False`；传了即为开启） |
| `--experience`                | 指定 experience 文件；留空则按 headless/enable\_cameras 自动选择 | 否（默认空字符串 `""`）         |
| `--rendering_mode`            | 渲染质量预设：`performance/balanced/quality`               | 否（默认 `balanced`）       |
| `--kit_args`                  | 透传给 Kit 的额外参数（一个字符串，空格分隔）                           | 否（默认空字符串 `""`）         |
| `--anim_recording_start_time` | 动画录制开始时间（秒）                                         | 否（默认 `0`）              |
| `--anim_recording_stop_time`  | 动画录制停止时间（秒）                                         | 否（默认 `10`）             |

