using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding
{
    public class EdgePoint
    {
        /**
         * Triangle which must be crossed to reach the next path point.
         */
        public Triangle toNode;

        /**
         * Triangle which was crossed to reach this point.
         */
        public Triangle fromNode;

        /**
         * Path edges connected to this point. Can be used for spline generation at some
         * point perhaps...
         */
        public List<TriangleEdge> connectingEdges = new List<TriangleEdge>();

        /**
         * The point where the path crosses an edge.
         */
        public TSVector point;

        public EdgePoint(TSVector point, Triangle toNode) {
            this.point = point;
            this.toNode = toNode;
        }
    }
}