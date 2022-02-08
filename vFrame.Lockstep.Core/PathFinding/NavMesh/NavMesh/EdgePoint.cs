using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh;

public class EdgePoint
{
    /**
     * Path edges connected to this point. Can be used for spline generation at some
     * point perhaps...
     */
    public List<TriangleEdge> connectingEdges = new();

    /**
         * Triangle which was crossed to reach this point.
         */
    public Triangle fromNode;

    /**
         * The point where the path crosses an edge.
         */
    public TSVector point;

    /**
         * Triangle which must be crossed to reach the next path point.
         */
    public Triangle toNode;

    public EdgePoint(TSVector point, Triangle toNode) {
        this.point = point;
        this.toNode = toNode;
    }
}