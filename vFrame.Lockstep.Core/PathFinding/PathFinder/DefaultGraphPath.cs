using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding;

public class DefaultGraphPath<TNode> : IGraphPath<TNode>
{
    public readonly List<TNode> Nodes;

    /**
     * Creates a {@code DefaultGraphPath} with no nodes.
     */
    public DefaultGraphPath() : this(new List<TNode>()) {
    }

    /**
     * Creates a {@code DefaultGraphPath} with the given capacity and no nodes.
     */
    public DefaultGraphPath(int capacity) : this(new List<TNode>(capacity)) {
    }

    /**
     * Creates a {@code DefaultGraphPath} with the given nodes.
     */
    public DefaultGraphPath(List<TNode> nodes) {
        Nodes = nodes;
    }

    public void Clear() {
        Nodes.Clear();
    }

    public int GetCount() {
        return Nodes.Count;
    }

    public void Add(TNode node) {
        Nodes.Add(node);
    }

    public TNode Get(int index) {
        return Nodes[index];
    }

    public void Reverse() {
        Nodes.Reverse();
    }
}