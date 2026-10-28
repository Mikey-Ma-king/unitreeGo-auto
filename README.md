# Go2 圆形与 S 型自动路线使用说明

本文说明本仓库新增的四个入口脚本：

| 文件 | 用途 |
|---|---|
| [`dp_circle.py`](./dp_circle.py) | Python 圆形路线入口 |
| [`dp_s_route.py`](./dp_s_route.py) | Python S 型往返路线入口 |
| [`run_dp_circle.sh`](./run_dp_circle.sh) | 配置 ROS 环境、启动遥控器转发和圆形路线 |
| [`run_dp_s_route.sh`](./run_dp_s_route.sh) | 配置 ROS 环境、启动遥控器转发和 S 型路线 |

两个 Python 入口都调用 [`dp_route_navigation.py`](./dp_route_navigation.py) 中的共享控制器。路线、速度、按键和故障处理均以共享控制器的当前代码为准。

> **重要安全提示**
>
> L1 和软件故障处理调用的是 Unitree Sport API 的 `StopMove` 请求。这是软件停止请求，不是硬件急停，也没有代码确认机器人已收到请求或已完全停住。操作时必须能独立使用实体急停或经验证的遥控器停止方式。请先低速、空旷、有人看护地测试；不要将导航模型视为可靠的安全避障装置。

## 1. 系统工作流

### 1.1 启动脚本做什么

两个 `.sh` 脚本的启动流程基本相同，只有最后执行的路线入口不同：

1. `set -euo pipefail` 令未处理的命令错误、未定义变量或管道错误导致脚本退出。
2. 查找并 source ROS 2 环境。默认发行版是 Foxy；若使用其他发行版，可在启动前设置 `ROS_DISTRO`。
3. 加载 Unitree ROS 2 工作空间。优先使用仓库内 `cyclonedds_ws/install/setup.bash`，其次使用 `$HOME/cyclonedds_ws/install/setup.bash`；两处都不存在就报错退出。
4. 若仓库中有 `.env/bin/activate`，则激活该 Python 虚拟环境。
5. 默认使用 `rmw_cyclonedds_cpp`。如果没有设置 `CYCLONEDDS_URI`，脚本尝试使用 `$HOME/cyclonedds_ws/cyclonedds.xml` 或仓库内 `cyclonedds_ws/src/cyclonedds.xml`。
6. 检查 Python/ROS 命令、路线入口、共享控制器、Sport API 封装、ONNX 文件及 ROS/Python 导入依赖。
7. 查询 RealSense 设备是否可发现；随后用全零输入、与导航节点相同的张量形状对 ONNX 模型执行一次 smoke inference，并记录执行提供程序及输出形状。
8. 在最多 20 秒内轮询 ROS 图，要求发现 `/lf/lowstate` 和 `/lf/sportmodestate`。超时则报错退出，且不会启动遥控器客户端。
9. 后台启动 [`tools/udp_controller_client.py`](./tools/udp_controller_client.py)，由它接收 UDP 遥控器数据并发布到 ROS 话题 `/udp_controller`。该客户端在初始化时还会发布一次 `StandDown`（趴下）请求；因此启动时要注意机器人姿态和周围空间。启动脚本会等待 1 秒并检查该子进程仍存活。
10. 以前台进程运行所选路线的 Python 入口。启动阶段会打印时间戳、检查阶段和失败原因；日志直接输出到终端，不会自动保存到文件。
11. Shell 的退出钩子尝试终止并等待 UDP 客户端。Python 导航程序正常退出或进入异常清理时，会在活动路线仍运行的情况下尝试发送 `StopMove`。

可通过 `ROUTE_TOPIC_DISCOVERY_TIMEOUT` 调整机器人话题发现等待时间（秒），默认 20 秒。例如：

```bash
ROUTE_TOPIC_DISCOVERY_TIMEOUT=40 ./run_dp_circle.sh
```

这些启动前检查可以发现 ROS 环境、文件、Python 依赖、RealSense 设备发现、模型加载/张量兼容及 DDS 话题发现方面的常见问题，但**不能**证明相机已经成功启动要求的 640×360/30 Hz 深度流、深度图内容有效、模型对真实图像输出合理动作、机器人已进入站立状态或 `StopMove` 一定有效。零输入 smoke inference 只验证模型可以加载并接受当前代码提供的输入形状，不验证避障质量。运行节点的 `Ready` 日志只表示 RealSense 启动后程序进入主循环，并不代表上述项目全部通过。

路线入口映射：

```text
run_dp_circle.sh  -> tools/udp_controller_client.py + dp_circle.py
run_dp_s_route.sh -> tools/udp_controller_client.py + dp_s_route.py
```

不要同时运行这两个启动脚本，也不要同时运行其他会向 `/api/sport/request` 发送运动指令的控制程序。多个控制源并发时，命令会互相覆盖或冲突。

### 1.2 Python 路线程序做什么

两个 Python 入口只选择路线类型：

```python
run_navigation("circle")
run_navigation("s_route")
```

共享控制器 [`dp_route_navigation.py`](./dp_route_navigation.py) 随后：

1. 加载 `diffphysics_inference/nav_large_v_20d_smooth_lag.onnx`，并用全零输入做模型初始化。
2. 订阅：
   - `/lf/sportmodestate`：机器人模式、位置、IMU 姿态；
   - `/lf/lowstate`：低层状态及电池电量；
   - `/udp_controller`：遥控器按键。
3. 创建 RealSense 深度流，配置为 `640 × 360`、`z16`、`30 Hz`。每轮 ROS 处理后读取帧，跳过奇数帧，使用 decimation 和 hole-filling 滤波，目标处理频率约为 15 Hz。
4. 路线未启动时只等待状态、相机和按键；收到 L2 且机器人模式为 `balanceStand` 后，根据当时的世界坐标位置和朝向生成路线。
5. 路线运行时，根据当前位姿推进路径点，选取前视目标；把目标方向变换到机体坐标系，并与预处理后的深度图一起输入导航 ONNX 模型。
6. 将模型输出限幅后，使用 `SportClient.Move` 发布 `/api/sport/request`；也发布 `/planner/v_cmd` 供可视化或其他订阅者使用。实际驱动当前狗的是本程序发布的 Sport `Move` 请求。

模型接收深度图并不等于已证明能避障。代码没有独立的“障碍物距离小于阈值就停车”规则；模型是否能避开具体障碍，必须在真实传感器和场地中验证。

## 2. 路线定义与参数

参数位于 [`dp_route_navigation.py`](./dp_route_navigation.py) 文件顶部。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `MODEL_PATH` | `diffphysics_inference/nav_large_v_20d_smooth_lag.onnx` | 两条路线共用的导航模型 |
| `TARGET_SPEED` | `0.5` | 送入模型的目标方向速度幅值上限 |
| `MAX_FORWARD_SPEED` | `0.5` | 模型输出前进速度 `vx` 的上限；圆形路线允许 `vx` 在 `[-0.5, 0.5]` |
| `MAX_SIDE_SPEED` | `0.3` | 侧向速度 `vy` 的绝对值上限 |
| `MAX_YAW_SPEED` | `0.7` | 偏航速度 `vyaw` 的绝对值上限 |
| `LOOKAHEAD_POINTS` | `3` | 路径跟踪的前视点数 |
| `WAYPOINT_REACHED_DISTANCE` | `0.55 m` | 当前路径点在此距离内时推进路径索引 |
| `STATE_TIMEOUT` | `1.0 s` | 机器人状态或深度帧的超时阈值 |
| `CIRCLE_RADIUS` | `10.0 m` | 圆形参考路线半径 |
| `CIRCLE_SPACING` | `0.4 m` | 圆周采样的近似间距 |
| `S_ROUTE_LENGTH` | `20.0 m` | S 型去程的前进距离 |
| `S_ROUTE_SPACING` | `0.4 m` | S 曲线采样间距 |
| `S_BEND_RADIUS` | `1.0 m` | 去程 S 弯的圆弧半径 |
| `S_TURN_RADIUS` | `1.0 m` | 两端掉头半圆的半径 |
| `S_BEND_PAIRS` | `10` | 去程 S 弯的弯道对数 |

### 2.1 圆形路线

[`make_circle_route()`](./dp_route_navigation.py) 在 L2 启动时读取当前位置作为圆周上的起点，并使用当前机体朝向定义圆的切线方向。采样点按约 0.4 米的圆周间距生成，路径索引首尾循环，因此路线持续重复，直到停止或发生已处理的故障。

圆的**参考几何半径**为 10 米；这是路线点的半径，不是对实机运动轨迹半径的保证。位置估计误差、速度控制误差、打滑和避障偏移都会使实际轨迹偏离参考圆。

### 2.2 S 型往返路线

去程由 10 对弯道构成；每个弯道对由两个相反方向的 90°、半径 1 米圆弧组成，因此每对提供约 2 米的前向位移，10 对形成约 20 米去程。终点连接半径 1 米的半圆，接到一条横向错开的返程线；返程沿 S 弯点列倒序，回到起点后再由另一个半圆接回去程，形成循环闭合路线。

参考路线每个 S 弯与两端掉头弧线的设计半径都是 1 米。路线追踪误差、机器人能力和模型为障碍所作的调整会影响实机的实际转弯半径。半径 1 米是紧弯，实际运行前要确认场地宽度、狗的转弯能力和低速跟踪效果。

S 路线将模型的前进速度限制为 `vx >= 0`，避免主动发出倒车速度。它仍允许模型输出受限的侧向速度，不能据此保证狗完全不会侧移。

## 3. 实际执行要求与按键顺序

### 3.1 启动前检查

每次启动前确认：

- 机器人周围足够空旷。10 米半径圆需要约 20 米直径的空间；S 路线约 20 米前进，并且返程线和半径 1 米的掉头弧还需要额外侧向空间。
- RealSense 已连接，深度流可用，视野无遮挡；模型文件存在。
- 主机与机器人网络及 CycloneDDS 网卡配置正确，`/lf/sportmodestate`、`/lf/lowstate` 正在更新。
- 没有其他导航/运动节点同时发布机器人运动命令。
- 操作者能立即使用独立硬件急停或已验证的遥控停止方式。
- 启动脚本会先请求机器人 `StandDown`，所以确保机器人周围安全，并预期它可能先趴下。

Shell 脚本默认 ROS Foxy：

```bash
./run_dp_circle.sh
./run_dp_s_route.sh
```

如果使用其他发行版，例如 Humble：

```bash
ROS_DISTRO=humble ./run_dp_circle.sh
ROS_DISTRO=humble ./run_dp_s_route.sh
```

执行权限不足时可用 `bash run_dp_circle.sh` 或 `bash run_dp_s_route.sh` 启动。

### 3.2 按键与操作顺序

按键定义在共享控制器中：

| 按键 | 作用 |
|---|---|
| **L2** | 仅当收到的机器人模式为 `balanceStand` 时启动所选路线；路线运行时再次按 L2 只会记录“已经运行”的警告，不会停止或切换路线。 |
| **L1** | 停止当前自动路线并发送一次 `StopMove` 请求。它不是硬件急停。 |
| **R1** | 若路线正在运行，先请求 `StopMove`，再发送 `RecoveryStand` 请求。RecoveryStand 是恢复站立请求，不是普通路线暂停键。 |
| **R2** | 若路线正在运行，先请求 `StopMove`，再发送 `StandDown` 请求（趴下）。 |

推荐操作顺序：

1. 运行**其中一个**对应的 `.sh` 文件。
2. 查看启动日志中的各项预检查通过信息；随后等待路线节点报告 RealSense 启动/`Ready`。`Ready` 本身不是首帧深度有效、机器人状态有效或模型避障正确的保证；还要检查状态报告及实际传感器数据。
3. 启动脚本的遥控器客户端会先发 `StandDown`。根据机器人当前状态，使用遥控器恢复到 `balanceStand`，并确认 `/lf/sportmodestate` 中模式为 1。
4. 确认环境和急停准备好后，短按 **L2**。程序检查状态新鲜且模式为 `balanceStand`，记录当前位置和姿态，生成路线并开始跟踪。
5. 观察机器人和日志。需要停止时短按 **L1**，并确认机器人确实停下；如果未停下，立即使用独立急停。
6. 结束程序可在终端按 Ctrl+C。正常处理 KeyboardInterrupt 时，活动路线的清理流程会尝试发 `StopMove`；停止情况仍需现场确认。

按键使用新按下沿触发，长按通常只触发一次；必须松开再按才能产生下一次按键事件。若同一遥控器消息同时包含多个按键，代码的 `if/elif` 顺序为 **L1 → L2 → R1 → R2**，优先执行先匹配的分支。

这两个控制脚本均未实现摇杆轴接管行走。L1 停止路线后，不代表本程序切换到手动行走。

## 4. 故障保护机制

圆形和 S 型使用同一个 `RouteNavigation` 类，因此以下故障处理相同，路线类型只影响几何路径和前向速度下限。

| 故障或操作 | 当前代码行为 | 说明 |
|---|---|---|
| 按 L1 | 将 `active=False`，向 `/api/sport/request` 发布一次 `StopMove`。 | 软件停止请求；没有响应确认或停车状态确认。 |
| 启动时状态未到或不新鲜 | 不启动路线，记录错误。要求位置/姿态/模式和低层状态最近 1 秒内有更新。 | 应先检查 DDS 话题与网络。 |
| 运行时机器人状态超过 1 秒未更新 | 0.1 秒定时器看门狗检测并调用停止逻辑。 | 包含运动状态和低层状态时间戳；不保证请求被机器人执行。 |
| 深度帧超过 1 秒未处理 | 看门狗调用停止逻辑。 | 深度获取等待设置为 1000 ms；超时异常会进入退出清理。 |
| 深度帧不存在 | 若路线活动，调用停止逻辑。 | 无效深度帧会导致本路线停止，不会继续沿旧命令规划。 |
| 深度数组为空或不是二维 | 调用停止逻辑。 | 只检查数组形状/空值，不评估图像内容质量。 |
| 位置不是有限值、IMU 四元数无效 | 忽略该运动状态消息；如果新状态因此持续缺失，状态看门狗随后停止路线。 | 四元数为零长度或含无效值会被拒绝。 |
| 模型动作维度不足或含 NaN/无穷值 | 调用停止逻辑，不发布该动作。 | ONNX Runtime 抛出的其他异常会传播到主循环退出清理。 |
| 模型给出过大的速度 | 在发布前对前进、侧向和偏航速度限幅。 | 速度限幅不是碰撞检测。S 路线额外禁止负 `vx`。 |
| 按 R1/R2 | 路线活动时先请求停止，再发布恢复站立/趴下请求。 | 这些操作可能改变机器人姿态，不应在不安全状态下随意触发。 |
| Python 主循环异常或 Ctrl+C | `finally` 在路线活动时调用停止，然后停止 RealSense、销毁 ROS 节点并关闭 ROS。 | 仅适用于 Python 能执行清理的退出；断电、强制终止等不保证运行 `finally`。 |
| Shell 退出 | EXIT/INT/TERM 钩子尝试终止 UDP 客户端。 | 不能据此保证运动程序在所有信号/系统故障场景下成功停止。 |

### 4.1 未实现的安全保护

当前代码**不提供**以下保证：

- 没有硬件急停实现，也没有确认 `StopMove` 被机器人接收或机器人已完全静止。
- 没有按深度阈值强制停车；没有单独的碰撞保护联锁。导航模型可能尝试绕障，但不能保证避障。
- 没有 UDP 遥控器链路超时看门狗。遥控消息丢失时，自动路线不会仅因此停止。
- 路线运行时没有持续检查模式是否仍为 `balanceStand`。
- 没有路线最大运行时长或到达终点自动停止；圆与 S 型都会持续循环。
- 不校验深度图是否被遮挡、全零、冻结或单位/标定是否正确。
- 停止请求不重试、不等待响应，也不进行制动效果确认。

出现机器人失控、避障异常、定位异常或软件停止未生效时，应立刻使用独立的硬件急停/经验证的遥控停止方式；不要依赖 L1 或模型进行最后一道安全保护。

## 5. 故障排查

| 现象 | 优先检查 |
|---|---|
| 启动时报 ROS setup 不存在 | 检查 `/opt/ros/<发行版>/setup.bash`，必要时设置正确的 `ROS_DISTRO`。 |
| 报 Unitree workspace 未构建 | 按项目 README 编译 `cyclonedds_ws`，确认 `install/setup.bash` 在仓库或 `$HOME/cyclonedds_ws` 下。 |
| 找不到 ONNX 模型 | 确认 `diffphysics_inference/nav_large_v_20d_smooth_lag.onnx` 存在，安装环境含 `onnxruntime`。 |
| 相机启动失败/无深度帧 | 检查 RealSense USB、权限、其他进程是否占用设备及 640×360 深度流支持情况。 |
| 按 L2 后提示非 balanceStand | 先用有效遥控器流程使狗进入平衡站立，再确认 `/lf/sportmodestate` 模式值为 1。 |
| 状态超时后路线停止 | 检查机器人网络、CycloneDDS 配置网卡名及 `/lf/sportmodestate`、`/lf/lowstate` 是否在持续发布。 |
| 导航能运行但路线偏离/不避障 | 先停止并排查位姿坐标系、IMU 朝向、相机视野和模型输入；不要提高速度来掩盖问题。 |

## 6. 相关实现

- 路线入口：[`dp_circle.py`](./dp_circle.py)、[`dp_s_route.py`](./dp_s_route.py)
- 启动环境与遥控转发：[`run_dp_circle.sh`](./run_dp_circle.sh)、[`run_dp_s_route.sh`](./run_dp_s_route.sh)、[`tools/udp_controller_client.py`](./tools/udp_controller_client.py)
- 路线生成、深度模型推理、按键、限幅和停止逻辑：[`dp_route_navigation.py`](./dp_route_navigation.py)
- Sport API 消息封装：[`sport_client.py`](./sport_client.py)
- Unitree ROS 2 配置与话题说明：[`README_zh.md`](./README_zh.md)

## 7. 旧版 `run.sh` 与 `run_dp.sh` 工作流

本节说明仓库原有的 [`run.sh`](./run.sh) 和 [`run_dp.sh`](./run_dp.sh)，包括它们启动的 Python 节点、按键、参数和保护边界。它们是两套不同的控制架构，不要与上面的圆形/S 型脚本混为一谈，也不要同时运行。

> **运行前的重要警告**
>
> 两个旧 `.sh` 都会进入 `/home/unitree/unitree_ros2`，执行 `git reset --hard`，然后执行 `git pull origin master`。`git reset --hard` 会丢弃该仓库中所有未提交的已跟踪文件修改；`git pull` 会更新代码。运行前先检查并备份本地修改，并确认工作目录、分支和远程仓库符合预期。不要把它们当作只负责启动程序的无副作用脚本。

### 7.1 `run.sh`：深度导航 + 关节控制器

#### 启动和数据流

`run.sh` 配置 ROS Foxy、CycloneDDS 和 CUDA 环境，激活 `/home/unitree/unitree_ros2/.env`，然后依次启动：

1. [`tools/udp_controller_client.py`](./tools/udp_controller_client.py)：接收 UDP 遥控器数据，发布 `/udp_controller`；其启动初始化还会发送一次 `StandDown` 请求。
2. [`df_odometer.py`](./df_odometer.py)：订阅 `/camera/infra1/image_rect_raw`、`/svo/pose_cam_0`、`/lf/lowstate` 和 `/udp_controller`；用 `diffphysics_inference/nav_large_v.onnx` 对图像与目标方向推理，并持续发布 `/planner/v_cmd`。目标位置由收到的 SVO 位置初始化，代码使用 `current_pos.z + 0.6` 和 `current_pos.x + 0.2` 作为目标坐标分量。这个节点依赖外部相机/SVO 节点提供输入；`run.sh` 本身没有启动这些节点。
3. [`base_controller_new.py`](./base_controller_new.py)：订阅 `/planner/v_cmd` 和 `/udp_controller`，将参考速度、IMU、关节状态等组成观测，输入 `diffphysics_inference/new_two_ckpt_mix.onnx`，再由 `go2_cpp_utils.LowControl` 发布低层关节控制。实际关节动作不是 `df_odometer.py` 直接发出的 Sport `Move`。

因此，`run.sh` 的避障/导航链路是：相机图像和 SVO 位姿 → `df_odometer.py` 导航模型 → `/planner/v_cmd` → `base_controller_new.py` 关节策略模型 → 低层关节命令。导航目标是固定的小偏移，不是用户指定的路线终点；脚本没有完成目标点后自动停止的完整导航逻辑。

`df_odometer.py` 中可见的相关设置包括模型 `nav_large_v.onnx`、输入目标向量缩放系数 `1.375`，以及目标坐标偏移 `(0.6, 0.2)`。`base_controller_new.py` 使用 `traj_q.txt` 作为站立/阻尼过渡轨迹，控制定时器为 100 Hz，`/planner/v_cmd` 超时阈值为 1 秒。代码还定义了机器人编号筛选状态；按 Select 会轮换 `dog_control`，用于限制当前节点实际控制的机器人编号。

#### 按键与建议顺序

| 按键 | 当前实现中的作用 |
|---|---|
| **R2** | 只在 `DAMPING` 状态启动站立过渡，沿 `traj_q.txt` 将关节目标逐步推进到站立姿态。 |
| **A** | 只在 `STAND` 且过渡系数 `pd_ratio > 0.5` 时进入 `ALGO`，开始关节策略控制。 |
| **L2** | 切换 `auto` 输入选择；它不启动/停止 `ALGO`。在 `ALGO` 中，`auto=True` 且近期收到 `/planner/v_cmd` 时使用导航模型的速度输入；否则使用遥控器摇杆输入。 |
| **L1** | 将 `dog_control` 设为 0 并强制 `auto=True`。它不是停止键，也不会直接退出 `ALGO` 或发送 `StopMove`。 |
| **R1** | 若处于 `ALGO`，切换到 `STAND` 并沿过渡轨迹返回阻尼姿态；完成轨迹后进入 `DAMPING`。 |
| **Select** | 在 `dog_control` 的 0 到 4 之间轮换控制编号；不负责启动导航。 |
| **L2 + B** | 同一按键消息中同时按下时，代码请求切回 `DAMPING`、重置轨迹，同时 L2 分支仍会切换 `auto`。不建议将它当作紧急停止组合键。 |

按键操作通常按 **R2 → 等待站立过渡完成 → A** 启动策略控制。默认 `auto=True`，若导航节点正在持续发布命令且 `v_flag` 有效，策略接收自动导航参考速度。按 L2 切为摇杆输入后，仍需处于 `ALGO`，策略才会用摇杆的前进/侧移/转向值控制。L1 只是恢复自动输入选择，不是停止机器人。需要退出策略时使用 R1，并确认机器人完成过渡。

按键状态由 `base_controller_new.py` 的按下沿逻辑处理；长按通常只触发一次，松开后再按才会再次触发。需要注意，`df_odometer.py` 自身收到 A 会重置目标初始化和模型隐状态，因此同一个 A 按键也可能同时触发两个节点各自的 A 逻辑。

#### 故障处理与限制

- `/planner/v_cmd` 超过 1 秒没有更新时，`base_controller_new.py` 清除 `v_flag`；当处于 ALGO 时，观测会退回摇杆输入，而不是发送 Sport `StopMove` 或自动进入阻尼。
- L1 不发停止请求；R1 是回到过渡/阻尼流程的控制操作，不是经确认的急停。
- 代码没有检查深度图是否冻结、全零或遮挡，也没有独立障碍距离阈值停车。
- 代码没有针对遥控器失联的停止看门狗；遥控消息中断时，摇杆缓存值可能继续作为策略输入。
- 状态、图像或模型异常是否能令机器人进入安全停止，取决于节点是否仍运行及低层控制接口行为；这里没有端到端的停止确认机制。
- `run.sh` 的 Ctrl+C 清理函数尝试终止其直接子进程，但不等于给机器人发送了有效停止请求；异常退出、断电或进程被强制终止都不能保证机器人已停稳。

### 7.2 `run_dp.sh`：深度模型 + Sport `Move` 导航

#### 启动和数据流

`run_dp.sh` 使用与 `run.sh` 相似的 ROS/CycloneDDS/CUDA 环境、虚拟环境和代码更新流程，但只启动 UDP 遥控转发以及 [`dp_test_turn_8.py`](./dp_test_turn_8.py)。它不启动 `df_odometer.py` 或 `base_controller_new.py`。

`dp_test_turn_8.py` 订阅 `/lf/lowstate`、`/lf/sportmodestate`、`/udp_controller`，并直接读取 RealSense 深度和彩色流。深度图以 `640 × 360`、`z16`、`30 Hz` 配置；循环跳过帧并使用 decimation、hole-filling 滤波，约以 15 Hz 处理。程序加载 `diffphysics_inference/nav_large_v_20d_smooth_lag.onnx`，把深度和相对目标方向输入模型，使用 `SportClient.Move` 直接向 `/api/sport/request` 发布速度，也发布 `/planner/v_cmd`。这里的 `/planner/v_cmd` 不是实际驱动狗的唯一链路；实际运动命令由本节点直接发 Sport `Move`。

主要参数在 [`dp_test_turn_8.py`](./dp_test_turn_8.py) 顶部：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `FWD_DISTANCE` | `5 m` | L2 启动时，相对当前位置和机体朝向生成的前方目标距离。 |
| `TARGET_SPEED` | `1` | 送入模型的目标向量速度幅值上限。 |
| `SINGLE_WAYPOINT_DURATION` | `12 s` | 普通目标段的计时长度；目标推进以时间为主。 |
| `SINGLE_WAYPOINT_DURATION_MINER` | `10 s` | A 返程计划的总时长计算参数。 |
| `TURN_TIME` | `3 s` | 目标切换时的预留原地转向时间。 |
| `TURN_ON_APPROACHED` | `False` | 关闭按接近目标距离切换；当前主要依靠计时推进目标。 |
| 自动计划时长 | `120 s` | L2 启动后，到时调用 `stop()`；实际运行还依赖深度循环继续处理。 |

L2 记录启动时的位姿，按当时的机体朝向生成“前方 5 米”及“启动位置”两个目标。按 A 会把目标索引设置为已记录的起点，尝试返程；必须先成功按 L2 记录起点。它不是任意路径规划，也不是可靠的精确返航/到点保证。普通计划持续最多约 120 秒，并在两个目标之间按时间轮换；这不应理解成只沿直线、精确到点后停下的导航任务。

#### 按键与建议顺序

| 按键 | 当前实现中的作用 |
|---|---|
| **L2** | 当机器人模式为 `balanceStand` 时启动前方 5 米目标与起点之间的自动计划；若已在计划中则不重复启动。 |
| **A** | 当模式为 `balanceStand` 且此前已记录起点时，启动面向起点的返程计划；没有有效起点记录时不启动。 |
| **L1** | 若计划活动，清除计划状态、关闭记录文件并发布一次 Sport `StopMove` 请求。 |
| **R1** | 发布 `RecoveryStand` 请求；处理代码没有先停止自动计划。 |
| **R2** | 发布 `StandDown` 请求；处理代码没有先停止自动计划。 |

建议顺序：启动 `run_dp.sh` → 等待深度/状态数据正常并确认机器人处于 `balanceStand` → 短按 L2 记录起点并开始自动计划 → 如需返程，短按 A → 需要结束时按 L1，现场确认机器人确已停下。按键回调按当前按键位处理，没有类似新路线控制器那样的按下沿去抖；保持按键时可能在多条遥控器消息中重复进入分支，但活动计划检查会阻止重复启动。

#### 故障处理与限制

- L1 调用 `StopMove` 并停止后续路线处理；这是软件请求，没有应答确认、停车确认或重试。
- 计划设置 120 秒的最大时长，但到时停止检查是在深度处理循环中进行；深度流卡住时，不保证能按时调用停止逻辑。
- `main()` 的 `finally` 会停止 RealSense、销毁 ROS 节点并关闭 ROS，但没有显式调用 `planner.stop()`；异常退出不能据此认为会发送 `StopMove`。
- 没有机器人状态/遥控器链路超时看门狗，没有持续检查 `balanceStand` 模式，也没有深度图冻结、遮挡或无效内容检测。
- 模型速度没有完整的统一限幅：侧向输出乘以 `0.3`，偏航使用 `tanh`；前向输出并未在该代码路径统一裁剪到明确上限。
- 目标切换主要靠 12 秒计时，接近目标切换开关关闭；位置误差、模型偏移、打滑和障碍绕行都可能造成路径与预期不一致。
- R1/R2 在计划期间不会先调用计划停止逻辑，可能造成姿态命令与自动运动命令并存或冲突。不要把它们用作计划运行中的停止键。
- 两个旧 `.sh` 的 Ctrl+C 清理同样只是尝试终止子进程，不构成机器人急停或停止确认。

### 7.3 两套旧流程的选择

| 项目 | `run.sh` | `run_dp.sh` |
|---|---|---|
| 运动控制链路 | ONNX 导航参考速度 → 关节策略 ONNX → 低层关节控制 | ONNX 导航速度 → Sport `Move` |
| 导航输入 | 外部提供的红外图像话题、SVO 位姿 | 本程序读取 RealSense 深度、机器人运动状态 |
| 主要导航行为 | 固定小偏移目标；路径终点和停止逻辑不完善 | 前方约 5 米与已记录起点间按时序切换，可用 A 请求返程 |
| 主要输入切换 | L2 在策略运行时切换导航参考速度/摇杆输入 | 没有摇杆手动运动接管；A/L2/L1控制计划 |
| 软件停止 | 没有 L1 `StopMove` 实现 | L1 发一次 `StopMove` |

两套旧流程都依赖模型和外部软硬件状态，不能视为安全认证的自主避障系统。测试前应先审查并处理启动脚本中的 `git reset --hard`，在空旷场地低速验证，并始终准备独立、已验证的硬件急停方式。
