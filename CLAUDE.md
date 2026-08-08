# vFrame.LockstepCore

Deterministic lockstep networking core (Unity-free) for multiplayer simulation. Provides fixed-point math, 2D physics (Farseer), grid/NavMesh pathfinding (A*), and RVO2 local avoidance.

## Quick Reference

| Topic | Location |
|-------|----------|
| [Workspace conventions](../../../CLAUDE.md) | Multi-package layout, Unity version, shared gotchas |
| README.md | Feature overview, quick start, usage examples |

## Package Structure

This is **not** a standard Unity `Assets/` project. Core source lives in `vFrame.Lockstep.Core/` (Unity-free C# library), while the UPM package payload lives in `Output/`.

```
vFrame.LockstepCore/
├── vFrame.Lockstep.Core/       # Core runtime (Unity-free, .NET Framework 4.7.1)
│   ├── Math/                    # FixedPoint, TSVector, TSQuaternion, TSMatrix
│   ├── FarseerPhysics/          # 2D physics (World, Body, Fixture, Joint, Factories)
│   ├── PathFinding/             # Grid graphs, NavMesh, A* pathfinding
│   ├── RVO2/                    # RVO2 Simulator (agent crowd movement, local avoidance)
│   └── TSTime.cs                # Engine-agnostic frame time tracking
├── Output/
│   ├── Runtime/                 # Unity runtime bridge (vFrame.Lockstep.Unity.asmdef)
│   ├── Editor/                  # Unity editor tools (vFrame.Lockstep.Core.Editor.asmdef)
│   └── package.json             # UPM package entry
└── Makefile                     # Build: produces vFrame.Lockstep.Core.dll
```

## Core Concepts

### Deterministic Math

All game-state values requiring cross-machine consistency must use `FixedPoint` (Q31.32 representation), not `float`.

```csharp
using vFrame.Lockstep.Core;

FixedPoint speed = FixedPoint.FromFloat(3.5f);
FixedPoint frameTime = FixedPoint.FromFloat(1f / 60f);
FixedPoint distance = speed * frameTime;

TSVector2 position = new TSVector2(1, 0);  // Uses FixedPoint internally
FixedPoint dot = TSVector2.Dot(a, b);
```

**Key types**: `FixedPoint`, `TSVector`, `TSVector2`, `TSVector4`, `TSQuaternion`, `TSMatrix`, `TSMatrix4x4`, `TSRandom`.

### 2D Physics (Farseer)

`World` manages the physics simulation; `BodyFactory` creates bodies.

```csharp
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Factories;

var world = new World(new TSVector2(0, -10));  // gravity
var body = BodyFactory.CreateCircle(world, radius: 1, density: 1, position: new TSVector2(0, 5));
body.BodyType = BodyType.Dynamic;

world.Step(FixedPoint.FromFloat(1f / 60f));
```

**Key types**: `World`, `Body`, `Fixture`, `BodyType`, `BodyFactory`, `FixtureFactory`, `JointFactory`, joints (`AngleJoint`, `DistanceJoint`, etc.).

### Grid Pathfinding

`GridGraph` builds nodes from `GridData`; `IndexedAStarPathFinder<TNode>` performs A* search.

```csharp
using vFrame.Lockstep.Core.PathFinding.Grid;

var gridData = new GridData {
    cols = 4, rows = 4,
    start = TSVector2.zero,
    width = 1,
    grids = new[] { 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 }  // 0 = blocked
};

var graph = new GridGraph(gridData, mask: 1);
graph.SetHypotenuseConnectionEnabled(true);
graph.RebuildConnections();

var finder = new IndexedAStarPathFinder<GridNode>(graph);
var path = new GridGraphPath();
bool found = finder.SearchPath(startNode, endNode, new GridHeuristic(), path);
```

**Key types**: `GridGraph`, `GridNode`, `GridData`, `GridGraphPath`, `IndexedAStarPathFinder<TNode>`, `IPathFinder`.

### RVO2 Local Avoidance

`Simulator` manages agents, obstacles, and time stepping for crowd movement.

```csharp
using vFrame.Lockstep.Core.RVO2;

var simulator = new Simulator();
simulator.setTimeStep(FixedPoint.FromFloat(1f / 10f));
simulator.setAgentDefaults(neighborDist: 15, maxNeighbors: 10, timeHorizon: 10,
                          radius: 10, maxSpeed: 0.5f, acceleration: 2, velocity: TSVector2.zero);

int agentId = simulator.addAgent(TSVector2.zero);
simulator.setAgentVelocity(agentId, TSVector2.right);
FixedPoint time = simulator.doStep();
TSVector2 position = simulator.getAgentPosition(agentId);
```

**Key types**: `Simulator`, `Agent`, `Obstacle`, `KdTree`.

### Frame Time Tracking

`TSTime` provides engine-agnostic frame counting and delta time.

```csharp
var time = new TSTime();
time.Init();
time.Update();

FixedPoint deltaTime = time.deltaTime;
int frameCount = time.frameCount;
```

## Build & Test

### Build from source

```bash
cd D:/Workspace/vFrame/vFrame.LockstepCore
make
```

Produces `Build/Release/Runtime/vFrame.Lockstep.Core.dll`, copied to `Output/Runtime/`.

**No runtime tests** — this is a pure computation library consumed by Unity projects. Verify by consuming the Output/ package in a Unity test scene.

## Gotchas

### Determinism Rules

- **Never use `float` or `double` for game state** — only `FixedPoint` and `TSVector*` types
- Fixed-point precision is 2^-32 (~2.3E-10); use `FixedPoint.FromFloat(float)` for conversions
- `Simulator.doStep()` runs **synchronously** to preserve determinism

### API Pitfalls

- `GridGraph` treats a cell as traversable when `(flag & mask) > 0` — the `grids` array flag values must match the constructor `mask`
- `Simulator.addAgent(TSVector2 position)` requires `setAgentDefaults(...)` to be called first, otherwise returns `-1`
- `World` exposes a static `Instance` — but at gameplay level, treat each `World` instance as explicitly owned simulation state

### Package Layout

- This repo does **not** use standard Unity `Assets/` structure
- Core source (`vFrame.Lockstep.Core/`) is a Unity-free C# project targeting .NET Framework 4.7.1
- `Output/` is the UPM package payload; asmdef files reference **precompiled DLLs**, not source
- Dependencies: `vFrame.Core.Lite.dll` (precompiled, in `Libraries/`)

## Third-Party Components

- **Farseer Physics Engine** — 2D physics runtime (Box2D-derived)
- **RVO2 Library** — local avoidance simulation (UNC Chapel Hill, Apache 2.0)
- **Jitter-style math implementations** — fixed-point math (MIT/X11)

Notices retained in respective source files.
