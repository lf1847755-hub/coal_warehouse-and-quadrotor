# 煤堆仓库 + 四旋翼无人机仿真（Isaac Sim 5.1 + Pegasus）

在 Isaac Sim 5.1 中程序化生成一个 48 × 30 m 的开放式煤堆仓库（8 个静态碰撞煤堆、钢结构、可隐藏屋顶），用 Pegasus Simulator 的 Iris 四旋翼做键盘手动飞行。飞控是自研的 Python 位置保持后端，**不依赖 ROS 2、DDS、PX4、ArduPilot 或 QGroundControl**。

## 环境要求

| 项目 | 要求 |
|---|---|
| 系统 | Windows 10/11 x64 |
| 显卡 | NVIDIA RTX，显存 ≥ 8 GB（8 GB 笔记本显卡可用 960×540 + DLSS 实时跑） |
| 驱动 | 支持 RTX 50 系的新驱动（592.82 验证通过） |
| 仿真器 | **Isaac Sim 5.1.0**（Windows 版；5.0 / 4.5 的 API 不兼容） |
| 扩展 | **Pegasus Simulator v5.1.0**（必须是配 Isaac 5.1 的版本） |
| Python | Isaac Sim 自带的 Kit Python 3.11，**不要用 conda / 系统 Python** |
| 依赖包 | `pymavlink`（安装 Pegasus 时自动装）；numpy / scipy 由 Isaac Sim 自带 |
| 不需要 | ROS 2、DDS、PX4、ArduPilot、QGroundControl |

## 快速开始

完整步骤（含每步验证方式与排错）见 [docs/安装与交接说明.md](docs/安装与交接说明.md)。

1. 克隆到 Isaac Sim 目录下，**必须与 `PegasusSimulator` 同级**：
   ```bat
   git clone https://github.com/lf1847755-hub/coal_warehouse-and-quadrotor.git "%ISAACSIM%\coal_warehouse"
   ```
2. 安装 Isaac Sim 5.1.0，把 Pegasus Simulator 5.1.0 放到 `%ISAACSIM%\PegasusSimulator`。
3. 在 `%ISAACSIM%\apps\isaacsim.exp.base.kit` 的 `[dependencies]` 首行加一行：
   ```toml
   "isaacsim.replicator.agent.core" = {}
   ```
4. 把 Pegasus 装进 Kit Python（editable 安装）：
   ```bat
   "%ISAACSIM%\python.bat" -m pip install -e "%ISAACSIM%\PegasusSimulator\extensions\pegasus.simulator"
   ```
5. 打 Windows 兼容补丁（两处：ROS 后端开关、跳过联网资产检查）：
   ```bat
   docs\apply-pegasus-patch.bat "%ISAACSIM%\PegasusSimulator\extensions\pegasus.simulator"
   ```
6. 打开 `launch_warehouse.bat`，把 `set ISAACSIM_PATH=E:\isaacsim` 改成本机实际路径。
7. 双击 `launch_warehouse.bat`（可自行建桌面快捷方式，目标指向它、工作目录设为本目录）。首次加载约 1–3 分钟。

## 目录结构

`coal_warehouse.usda` 用相对路径引用 Pegasus 的 Iris 资产，所以两个目录必须是兄弟：

```
%ISAACSIM%\
├─ apps\isaacsim.exp.base.kit
├─ PegasusSimulator\extensions\pegasus.simulator\...
└─ coal_warehouse\            ← 本仓库
   └─ coal_warehouse.usda     →  @../PegasusSimulator/extensions/.../Iris/iris.usd@
```

换盘符可以，改变层级会让无人机资产丢失。

## 场景保存与加载

正常启动会读取本目录的 `coal_warehouse.usda`，不再重新生成或覆盖你的场景。只有场景文件不存在时才生成初始仓库。启动时会重新添加无人机，并让它回到起飞坪。

编辑流程：

1. 点击右侧 **Stop / Edit**（或 Isaac Sim 左侧 Stop），确认状态为 STOPPED。
2. 在 Stage / Property 面板中调整煤堆位置、大小、材质，或添加物体。
3. 点击右侧 **Save scene + backup**。看到 Scene saved 才表示保存成功。
4. 点击 **Play / Resume** 重新试飞；关闭后下次启动会读取这次保存的修改。

每次保存都会在 `backups` 文件夹保留上一个版本，文件名含时间戳，保存失败不会覆盖原文件。
暂停 PAUSED 不等于停止：保存按钮只在 STOPPED 时可用，避免把飞行中的物理状态保存成场景。
请使用右侧保存按钮保存到本项目；Isaac 的 File / Save As 可以另存副本，但副本不会自动成为下次启动的场景。
关闭前请主动保存，未点击保存的修改不会自动写入项目文件。

**Restore default warehouse...** 是独立的恢复默认操作：仅在停止状态可用，确认后先保存当前编辑并备份，再恢复最初的 8 个煤堆与仓库。`/World/CoalWarehouse` 外自行添加的对象会保留。
需要恢复备份时，先关闭程序，保留当前文件副本，再把选中的 backups 文件复制到本目录并命名为 `coal_warehouse.usda`。

## 播放、暂停、停止、重置

- **Play / Resume**：暂停后从原位置继续；停止后重新初始化无人机，可以再次起飞。
- **Pause**：冻结当前飞行位置，恢复播放后继续，不会把空中无人机重置到起点。
- **Stop / Edit**：停止并关闭键盘飞控，持续保持停止，方便编辑场景。
- **Reset drone [R]**：无人机回到起飞坪，仓库修改保留；原来在播放、暂停或停止，重置后仍保持该状态。

Isaac Sim 左侧原生播放、暂停和停止按钮也支持这些流程。

## 键盘飞行

切换英文输入法，点击右侧 **Enable flight keyboard**，再点击三维视口内部。
等面板显示 **KEYBOARD ACTIVE** 后操作。第一次启动和停止后，键盘飞控默认关闭。
点击 Stage / Property / 搜索框或切换到其他程序时，飞行键会失效并清除移动指令；回到视口后先松开所有飞行键，再重新按下。
Ctrl、Alt、Shift 等组合键不会作为飞行指令。

| 按键 | 功能 |
|---|---|
| T | 起飞到 2 米 |
| W / S | 按无人机朝向前进 / 后退 |
| A / D | 左右平移 |
| Q / E | 左右转向 |
| ↑ / ↓ | 上升 / 下降 |
| H | 设定当前位置为悬停目标 |
| L | 自动缓慢降落，着地后停桨 |
| R | 重置到起飞坪 |
| F / O | 跟随视角 / 仓库总览 |
| V | 隐藏或显示屋顶；碰撞仍保留 |

松开移动按键会减速并悬停。速度上限 1.5 米/秒，高度上限 6.5 米。
撞击后若出现 FAULT，使用 Reset drone。暂停或停止时请使用面板按钮，飞行快捷键不会响应。
起飞、悬停和降落按钮不要求开启键盘飞控，但要求仿真正在播放。
关闭 Isaac Sim 窗口即可退出。不要在飞行中修改无人机的 Stage 变换。

## 验收演示顺序

1. 点击 Overview 展示开放仓库、8 个煤堆、钢结构和中央通道。
2. 点击 Follow，启用键盘并聚焦三维视口，按 T 起飞。
3. 按 W 沿通道前进；使用 A/D 和 Q/E 展示平移和转向。
4. 在煤堆间隙穿梭，松开按键展示悬停。
5. 返回空旷地面，按 L 降落。

## 文件说明

| 文件 | 作用 |
|---|---|
| `warehouse_app.py` | 启动、场景加载、飞行验收入口 |
| `flight_ui.py` | 控制面板、焦点保护、播放/暂停/停止/重置、保存操作 |
| `scene_storage.py` | 只读加载、原子保存和备份；保存环境，不保存运行中的无人机 |
| `manual_backend.py` | Pegasus 辅助飞控，输出旋翼转速 |
| `scene_builder.py` | 仓库、煤堆、材质、碰撞的参数化建模 |
| `coal_warehouse.usda` | 编辑后的仓库环境。单独打开只加载场景；手动飞控需使用启动脚本 |
| `scene_metadata.json` | 初始设计参数。编辑后的几何以 USD 为准 |
| `logs/runtime_metadata.json` | 本次启动的仿真与无人机参数 |
| `logs/acceptance_result.json` | 程序化飞行和碰撞体射线检查结果 |
| `logs/workflow_result.json` | 保存、加载、备份、时间线与输入门禁回归检查结果 |
| `docs/` | 安装与交接说明、Pegasus 补丁、打补丁脚本 |
| `backups/` | 显式保存前的场景备份（运行后生成） |

## 命令行参数

```bat
REM 无界面自检（飞行 + 碰撞体检查）
"%ISAACSIM%\python.bat" warehouse_app.py --self-test

REM 保存 / 加载 / 备份 / 输入门禁回归
"%ISAACSIM%\python.bat" warehouse_app.py --workflow-test

REM 空场景（只放地面 + 无人机）
"%ISAACSIM%\python.bat" warehouse_app.py --empty

REM 指定其他 USD 场景
"%ISAACSIM%\python.bat" warehouse_app.py --scene D:\path\to\scene.usda
```

## 常见问题

- **必须用 `python.bat` 启动**：它才会把 Isaac 的 `site-packages`、`exts` 与 DLL 目录加进环境；用系统 Python 会报找不到 `isaacsim` / `pxr`。
- **`import pegasus` 失败**：Pegasus 没按第 4 步装进 Kit Python，或版本不是 5.1.0。
- **启动日志出现 `ROS2 package not installed. ROS2Backend will not be available`**：正常现象，补丁故意禁用了 ROS 后端，本示例不需要 ROS 2。
- **场景里没有无人机 / 资产丢失**：`coal_warehouse` 与 `PegasusSimulator` 不在同一层级，或 Pegasus 被移动过。
- **显存不足 / 掉帧**：调低 `warehouse_app.py` 里的 `width` / `height`，或把 `--/rtx/post/dlss/execMode` 改为更偏性能的档位。
- **想接 ROS 2**：去掉 `PEGASUS_DISABLE_ROS2=1`，并在 Isaac Sim 里启用 `isaacsim.ros2.bridge` 扩展；与本项目当前默认配置互不冲突。

## 已配置范围与限制

Isaac Sim 5.1.0 + 官方 Pegasus Simulator v5.1.0 + Iris 无人机。飞行使用 Pegasus 旋翼推力与 PhysX 动力学、自定义 Python 位置保持控制器。
当前模式不需要 PX4、QGroundControl 或 ROS 2；如果要做软件在环或接真实遥控器，需要另行集成。
针对 RTX 5060 Laptop 8 GB / 16 GB 内存采用实时渲染、DLSS 性能档、960×540 渲染分辨率、CPU 120Hz 物理和单无人机；默认煤堆共 3840 个三角形。
Pegasus 官方未测试 Windows，本仓库保留两处兼容调整：跳过本地场景不需要的远程资产根目录检查、用 `PEGASUS_DISABLE_ROS2=1` 跳过 ROS 后端。两处改动见 [docs/pegasus-5.1.0-local-changes.patch](docs/pegasus-5.1.0-local-changes.patch)。
仓库默认 48×30 米，煤堆为静态碰撞网格，不模拟煤粒散落。正常飞行通过推力实现，重置会把无人机恢复至起点。

## 参考

- Pegasus Simulator 文档：<https://pegasussimulator.github.io/PegasusSimulator/>
- Isaac Sim 5.1 文档：<https://docs.isaacsim.omniverse.nvidia.com/>
