namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IConnection<out TNode> where TNode: INode
    {
        FP GetCost();
        TNode GetFromNode();
        TNode GetToNode();
    }
}