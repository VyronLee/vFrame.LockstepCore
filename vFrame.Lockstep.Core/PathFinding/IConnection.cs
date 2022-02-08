namespace vFrame.Lockstep.Core.PathFinding;

public interface IConnection<out TNode>
{
    /**
     * Returns the non-negative cost of this connection
     */
    FixedPoint GetCost();

    /**
     * Returns the node that this connection came from
     */
    TNode GetFromNode();

    /**
     * Returns the node that this connection leads to
     */
    TNode GetToNode();
}