namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IPathFinder<N>
    {
        bool SearchPath(N startNode, N endNode, IHeuristic<N> heuristic, IGraphPath<IConnection<N>> outPath);

        bool SearchNodePath(N startNode, N endNode, IHeuristic<N> heuristic, IGraphPath<N> outPath);
    }
}