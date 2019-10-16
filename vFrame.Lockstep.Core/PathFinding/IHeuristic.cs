namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IHeuristic<in TNode> where TNode: INode
    {
        FP Estimate(TNode node, TNode endNode);
    }
}