[English](README.md) | [简体中文](README.zh-CN.md)

# vFrame Lockstep Core

A lockstep runtime for deterministic multiplayer simulation, providing fixed-point math, 2D physics, pathfinding, and RVO-based local avoidance.

## Scope

`vFrame.LockstepCore` bundles two cohesive layers:

1. **Fixed-point math** (`Math/`): deterministic `FixedPoint` arithmetic plus `TSVector`/`TSVector2`/`TSVector4`/`TSQuaternion`/`TSMatrix`. Usable **independently** of lockstep — any code needing deterministic floating-point math can depend on just this layer.
2. **Lockstep + simulation primitives**: 2D physics (Farseer), A* pathfinding (grid + NavMesh), and RVO2 local avoidance.

Most users consume both together, so they ship as one package. If you only need the deterministic math, reference the package and use the `Math/` types directly.

## Features

- Deterministic `FixedPoint` arithmetic based on Q31.32 representation.
- Core math types such as `TSVector`, `TSVector2`, `TSQuaternion`, and `TSMatrix`.
- A 2D physics runtime centered on `World`, `Body`, fixtures, and joints.
- A* grid pathfinding through `GridGraph` and `IndexedAStarPathFinder<TNode>`.
- NavMesh data structures and funnel-based path smoothing utilities.
- RVO2 crowd movement and local avoidance simulation through `Simulator`.
- Unity bridge components such as `FSWorldComponent` and `FSBodyComponent`.
- A standalone Editor assembly for authoring and debugging helpers.

## Requirements

- Minimum Unity version declared in `Output/package.json`: `2018.4`
- Core runtime project `vFrame.Lockstep.Core` targets .NET Framework 4.7.1
- Depends on `com.vyronlee.vframe.core` `1.0.3`

## Installation

### Install as a local UPM package

The package entry file is located at `Output/package.json`.

Add the following to the consumer project's `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.vyronlee.vframe.lockstep.core": "file:../vFrame.LockstepCore/Output"
  }
}
```

You can also choose `Add package from disk...` in Unity Package Manager and select:

- `Output/package.json`

### Build the runtime from source

```bash
make
```

This produces `vFrame.Lockstep.Core.dll`, which is then referenced by the asmdefs under `Output/Runtime` and `Output/Editor`.

## Quick Start

### Deterministic math

```csharp
using vFrame.Lockstep.Core;

FixedPoint speed = FixedPoint.FromFloat(3.5f);
FixedPoint frameTime = FixedPoint.FromFloat(1f / 60f);
FixedPoint distance = speed * frameTime;

TSVector start = TSVector.FromFloat(0f, 0f, 0f);
TSVector end = TSVector.FromFloat(10f, 0f, 0f);
TSVector direction = (end - start).normalized;
```

### Grid pathfinding

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

## Usage

### Fixed-point numbers and vectors

For any game-state values that must remain consistent across machines, prefer `FixedPoint` over `float`.

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

### Run a 2D physics step

Use `World` to manage the physics world and `BodyFactory` to create rigid bodies.

```csharp
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Factories;

var world = new World(new TSVector2(0, -10));
var body = BodyFactory.CreateCircle(world, 1, 1, new TSVector2(0, 5));
body.BodyType = BodyType.Dynamic;

world.Step(FixedPoint.FromFloat(1f / 60f));
```

In Unity, you can drive the same runtime through bridge-assembly types such as `FSWorldComponent`, `FSBodyComponent`, shape components, and joint components.

### Run grid pathfinding

`GridGraph` builds nodes from `GridData`, and `IndexedAStarPathFinder<TNode>` performs the search.

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

### Use RVO2 for local avoidance

`Simulator` manages agents, obstacles, and time stepping.

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

### Track frame time

`TSTime` provides engine-agnostic frame counting and delta time.

```csharp
var time = new TSTime();
time.Init();
time.Update();

FixedPoint deltaTime = time.deltaTime;
int frameCount = time.frameCount;
```

## Architecture Overview

- `vFrame.Lockstep.Core/Math`: deterministic numeric and transform types such as `FixedPoint`, `TSVector`, and `TSQuaternion`
- `vFrame.Lockstep.Core/FarseerPhysics`: physics runtime, collision detection, factories, controllers, and joints
- `vFrame.Lockstep.Core/PathFinding`: grid graphs, NavMesh primitives, and A* interfaces and implementations
- `vFrame.Lockstep.Core/RVO2`: agent simulation, obstacles, kd-tree queries, and stepping logic
- `vFrame.Lockstep.Core/TSTime.cs`: frame-time helper for deterministic loops
- `Output/Runtime/vFrame.Lockstep.Unity.asmdef`: Unity runtime bridge assembly over the compiled DLLs
- `Output/Editor/vFrame.Lockstep.Core.Editor.asmdef`: editor-only helper and debugging assembly

## Notes

- `GridGraph` treats a cell as traversable when `(flag & mask) > 0`, so the flag values in `grids` must be used consistently with the constructor `mask`
- `Simulator.addAgent(TSVector2 position)` requires `setAgentDefaults(...)` to be called first, otherwise it returns `-1`
- `Simulator.doStep()` runs synchronously to preserve deterministic results
- `World` exposes a static `Instance`, but at the gameplay level each `World` instance should still be treated as explicitly owned simulation state
- This sub-repository does not use the standard Unity `Assets/` layout; core source lives in `vFrame.Lockstep.Core`, while the UPM package payload lives in `Output`

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
The repository also contains or derives from third-party components such as Farseer Physics, Jitter-style math implementations, and RVO2 source files; their notices are retained in the corresponding source files.
