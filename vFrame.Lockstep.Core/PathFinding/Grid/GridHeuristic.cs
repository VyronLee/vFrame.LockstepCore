namespace vFrame.Lockstep.Core.PathFinding.Grid
{
    public class GridHeuristic : IHeuristic<GridNode>
    {
        public FixedPoint Estimate(GridNode node, GridNode endNode) {
            return (node.center - endNode.center).sqrtMagnitude;
        }
    }
}