namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IPathFinder<TNode> where TNode: INode
    {
        bool SearchPath(TNode startNode, TNode endNode,
            IHeuristic<TNode> heuristic, IPath<TNode> path);
    }
}