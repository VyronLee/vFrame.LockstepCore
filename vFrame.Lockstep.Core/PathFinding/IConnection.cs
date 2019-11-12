using System;
using System.Collections.Generic;


namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IConnection<N>
    {
        /** Returns the non-negative cost of this connection */
        FixedPoint GetCost();

        /** Returns the node that this connection came from */
        N GetFromNode();

        /** Returns the node that this connection leads to */
        N GetToNode();
    }
}