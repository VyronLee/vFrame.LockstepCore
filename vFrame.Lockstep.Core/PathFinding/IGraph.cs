using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding;

public interface IGraph<TNode>
{
    List<IConnection<TNode>> GetConnections(TNode fromNode);
}