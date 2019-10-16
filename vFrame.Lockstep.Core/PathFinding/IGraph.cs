using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IGraph<TNode> where TNode: INode
    {
        TNode GetNode(int index);
        int GetNodeCount();
        List<IConnection<TNode>> GetConnections(TNode fromNode);
    }
}