namespace vFrame.Lockstep.Core.PathFinding;

public interface IGraphPath<TNode>
{
    /**
     * Returns the number of items of this path.
     */
    int GetCount();

    /**
     * Returns the item of this path at the given index.
     */
    TNode Get(int index);

    /**
     * Adds an item at the end of this path.
     */
    void Add(TNode node);

    /**
     * Clears this path.
     */
    void Clear();

    /**
     * Reverses this path.
     */
    void Reverse();
}