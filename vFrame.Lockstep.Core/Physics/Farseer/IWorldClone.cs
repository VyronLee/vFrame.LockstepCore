using System;
using vFrame.Lockstep.Core.Physics2D;

namespace vFrame.Lockstep.Core
{
    public interface IWorldClone
    {
        string checksum { get; }

        void Clone(IWorld iWorld);

        void Clone(IWorld iWorld, bool doChecksum);

        void Restore(IWorld iWorld);
    }
}