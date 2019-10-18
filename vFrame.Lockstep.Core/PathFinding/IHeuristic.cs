namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IHeuristic<in TNode> where TNode: INode
    {
        FixedPoint Estimate(TNode node, TNode endNode);
    }
}