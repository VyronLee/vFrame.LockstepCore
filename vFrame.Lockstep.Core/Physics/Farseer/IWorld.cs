using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.Physics2D
{
    public interface IWorld
    {
        List<IBody> Bodies();
    }
}