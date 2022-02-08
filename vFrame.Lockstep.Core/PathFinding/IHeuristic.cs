namespace vFrame.Lockstep.Core.PathFinding;

public interface IHeuristic<in TNode>
{
    /**
     * Calculates an estimated cost to reach the goal node from the given node.
     * @param node the start node
     * @param endNode the end node
     * @return the estimated cost
     */
    FixedPoint Estimate(TNode node, TNode endNode);
}