namespace vFrame.Lockstep.Core.PathFinding
{
    public interface IPath<out TNode> where TNode: INode
    {
        int GetCount();

        TNode GetAt(int index);
    }
}