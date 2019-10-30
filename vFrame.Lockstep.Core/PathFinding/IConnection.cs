namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IConnection<out TNode> where TNode : INode
    {
        FixedPoint GetCost();
        TNode GetFromNode();
        TNode GetToNode();
    }
}