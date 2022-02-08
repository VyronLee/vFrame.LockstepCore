namespace vFrame.Lockstep.Core.PathFinding;

public interface IIndexedGraph<TNode> : IGraph<TNode>
{
    /**
     * Returns the unique index of the given node.
     * @param node the node whose index will be returned
     * @return the unique index of the given node.
     */
    int GetIndex(TNode node);

    /**
     * Returns the number of nodes in this graph.
     */
    int GetNodeCount();
}