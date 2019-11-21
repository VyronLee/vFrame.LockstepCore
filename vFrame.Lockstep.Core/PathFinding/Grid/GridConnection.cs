namespace vFrame.Lockstep.Core.PathFinding.Grid
{
    public class GridConnection : IConnection<GridNode>
    {
        private readonly GridNode _from;
        private readonly GridNode _to;

        public GridConnection(GridNode from, GridNode to) {
            _from = from;
            _to = to;
        }

        public FixedPoint GetCost() {
            return (_to.center - _from.center).sqrtMagnitude;
        }

        public GridNode GetFromNode() {
            return _from;
        }

        public GridNode GetToNode() {
            return _to;
        }
    }
}