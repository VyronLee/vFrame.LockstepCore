# vFrame Lockstep Core

[English](README.md) | [简体中文](README.zh-CN.md)

用于多人确定性模拟的帧同步运行时，提供定点数数学、2D 物理、寻路以及基于 RVO 的局部避障能力。

## 范围（Scope）

`vFrame.LockstepCore` 打包了两层内聚的功能：

1. **定点数数学**（`Math/`）：确定性的 `FixedPoint` 运算，以及 `TSVector`/`TSVector2`/`TSVector4`/`TSQuaternion`/`TSMatrix` 类型。可**独立于帧同步**使用——任何需要确定性浮点运算的代码都可以只依赖这一层。
2. **帧同步与模拟原语**：2D 物理（Farseer）、A* 寻路（网格 + NavMesh）以及 RVO2 局部避障。

大多数用户会同时使用两者，因此作为一个包一起发布。如果你只需要确定性数学，引用本包并直接使用 `Math/` 下的类型即可。

## 特性

- 基于 Q31.32 表示的确定性 `FixedPoint` 定点数运算
- 提供 `TSVector`、`TSVector2`、`TSQuaternion`、`TSMatrix` 等核心数学类型
- 提供以 `World`、`Body`、夹具和关节为核心的 2D 物理运行时
- 通过 `GridGraph` 和 `IndexedAStarPathFinder<TNode>` 提供 A* 网格寻路
- 包含 NavMesh 数据结构与漏斗路径优化工具
- 通过 `Simulator` 提供 RVO2 群体移动与局部避障模拟
- 提供 `FSWorldComponent`、`FSBodyComponent` 等 Unity 桥接组件
- 提供独立的 Editor 程序集用于编辑和调试辅助

## 依赖与要求

- `Output/package.json` 中声明的 Unity 最低版本为 `2018.4`
- 核心运行时项目 `vFrame.Lockstep.Core` 目标框架为 .NET Framework 4.7.1
- 依赖 `com.vyronlee.vframe.core` `1.0.3`

## 安装

### 作为本地 UPM 包安装

包入口文件位于 `Output/package.json`。

在使用方项目的 `Packages/manifest.json` 中加入：

```json
{
  "dependencies": {
    "com.vyronlee.vframe.lockstep.core": "file:../vFrame.LockstepCore/Output"
  }
}
```

也可以在 Unity Package Manager 中使用 `Add package from disk...`，然后选择：

- `Output/package.json`

### 从源码构建运行时

```bash
make
```

构建后会生成 `vFrame.Lockstep.Core.dll`，并由 `Output/Runtime` 与 `Output/Editor` 下的 asmdef 进行引用。

## 快速开始

### 确定性数学运算

```csharp
using vFrame.Lockstep.Core;

FixedPoint speed = FixedPoint.FromFloat(3.5f);
FixedPoint frameTime = FixedPoint.FromFloat(1f / 60f);
FixedPoint distance = speed * frameTime;

TSVector start = TSVector.FromFloat(0f, 0f, 0f);
TSVector end = TSVector.FromFloat(10f, 0f, 0f);
TSVector direction = (end - start).normalized;
```

### 网格寻路

```csharp
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.PathFinding.Grid;

var gridData = new GridData {
    cols = 4,
    rows = 4,
    start = TSVector2.zero,
    width = 1,
    grids = new[] {
        1, 1, 1, 1,
        1, 0, 0, 1,
        1, 1, 1, 1,
        1, 1, 1, 1,
    }
};

var graph = new GridGraph(gridData, 1);
var finder = new IndexedAStarPathFinder<GridNode>(graph);
var path = new GridGraphPath();

bool found = finder.SearchPath(
    graph.GetNode(0, 0),
    graph.GetNode(3, 3),
    new GridHeuristic(),
    path);
```

## 使用说明

### 定点数与向量

对所有需要跨机器保持一致的游戏状态值，优先使用 `FixedPoint` 而不是 `float`。

```csharp
FixedPoint angle = FixedPoint.FromFloat(45f);
FixedPoint sine = FixedPoint.Sin(angle);
FixedPoint cosine = FixedPoint.Cos(angle);
FixedPoint length = FixedPoint.Sqrt(9);

TSVector a = TSVector.FromFloat(1f, 0f, 0f);
TSVector b = TSVector.FromFloat(0f, 1f, 0f);
FixedPoint dot = TSVector.Dot(a, b);
TSVector cross = TSVector.Cross(a, b);
TSVector blended = TSVector.Lerp(a, b, FixedPoint.FromFloat(0.5f));
```

### 执行 2D 物理步进

使用 `World` 管理物理世界，使用 `BodyFactory` 创建刚体。

```csharp
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Factories;

var world = new World(new TSVector2(0, -10));
var body = BodyFactory.CreateCircle(world, 1, 1, new TSVector2(0, 5));
body.BodyType = BodyType.Dynamic;

world.Step(FixedPoint.FromFloat(1f / 60f));
```

在 Unity 中，可通过桥接程序集中的 `FSWorldComponent`、`FSBodyComponent`、各类形状组件和关节组件驱动同一套运行时。

### 执行网格寻路

`GridGraph` 根据 `GridData` 构建节点，`IndexedAStarPathFinder<TNode>` 负责搜索。

```csharp
var graph = new GridGraph(gridData, 1);
graph.SetHypotenuseConnectionEnabled(true);
graph.RebuildConnections();

var startNode = graph.GetNode(new TSVector2(0.5f, 0.5f));
var endNode = graph.GetNode(3, 3);
var path = new GridGraphPath();

bool found = finder.SearchPath(startNode, endNode, new GridHeuristic(), path);
int stepCount = path.GetCount();
```

### 使用 RVO2 做局部避障

`Simulator` 负责管理代理、障碍物和时间步进。

```csharp
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.RVO2;

var simulator = new Simulator();
simulator.setTimeStep(FixedPoint.FromFloat(1f / 10f));
simulator.setAgentDefaults(15, 10, 10, 10, 0.5f, 2, TSVector2.zero);

int agentId = simulator.addAgent(TSVector2.zero);
simulator.setAgentVelocity(agentId, TSVector2.right);
FixedPoint time = simulator.doStep();
TSVector2 position = simulator.getAgentPosition(agentId);
```

### 跟踪帧时间

`TSTime` 提供与引擎无关的帧计数和增量时间。

```csharp
var time = new TSTime();
time.Init();
time.Update();

FixedPoint deltaTime = time.deltaTime;
int frameCount = time.frameCount;
```

## 架构概览

- `vFrame.Lockstep.Core/Math`：`FixedPoint`、`TSVector`、`TSQuaternion` 等确定性数值与变换类型
- `vFrame.Lockstep.Core/FarseerPhysics`：物理运行时、碰撞检测、工厂、控制器与关节
- `vFrame.Lockstep.Core/PathFinding`：网格图、NavMesh 基元以及 A* 接口与实现
- `vFrame.Lockstep.Core/RVO2`：代理模拟、障碍物、kd-tree 查询与步进逻辑
- `vFrame.Lockstep.Core/TSTime.cs`：用于确定性循环的帧时间辅助类
- `Output/Runtime/vFrame.Lockstep.Unity.asmdef`：基于编译后 DLL 的 Unity 运行时桥接程序集
- `Output/Editor/vFrame.Lockstep.Core.Editor.asmdef`：编辑器专用的辅助与调试程序集

## 注意事项

- `GridGraph` 通过 `(flag & mask) > 0` 判断格子是否可通行，因此 `grids` 中的标记值必须与构造函数中的 `mask` 配套使用
- `Simulator.addAgent(TSVector2 position)` 依赖先调用 `setAgentDefaults(...)`，否则会返回 `-1`
- `Simulator.doStep()` 当前采用同步执行，以保证确定性结果一致
- `World` 暴露了静态 `Instance`，但在业务层仍应将每个 `World` 实例视为显式持有的模拟状态
- 该子仓库不是标准的 Unity `Assets/` 结构；核心源码位于 `vFrame.Lockstep.Core`，UPM 包内容位于 `Output`

## 许可证

本项目基于 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 许可协议发布。
仓库中还包含或派生自多个第三方组件，例如 Farseer Physics、Jitter 风格数学实现以及 RVO2 源码；对应声明保留在各自源文件中。
