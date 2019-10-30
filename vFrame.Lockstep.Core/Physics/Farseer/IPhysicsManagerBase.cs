using System;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.Physics2D;

public interface IPhysicsManagerBase
{
    void Init();

    void UpdateStep();

    IWorld GetWorld();

    IWorldClone GetWorldClone();

    void RemoveBody(IBody iBody);
}