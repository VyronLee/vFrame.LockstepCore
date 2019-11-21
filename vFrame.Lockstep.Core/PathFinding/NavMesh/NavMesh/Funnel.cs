using vFrame.Lockstep.Core.PathFinding.NavMesh.Geometry;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh
{
    public class Funnel
    {
        public Plane leftPlane = new Plane(); // 左平面，高度为y轴
        public Plane rightPlane = new Plane();
        public TSVector leftPortal = new TSVector(); // 路径左顶点，
        public TSVector rightPortal = new TSVector(); // 路径右顶点
        public TSVector pivot = new TSVector(); // 漏斗点，路径的起点或拐点

        public void setLeftPlane(TSVector pivot, TSVector leftEdgeVertex) {
            leftPlane.set(pivot, pivot.Add(TSVector.up), leftEdgeVertex);
            leftPortal = leftEdgeVertex;
        }

        public void setRightPlane(TSVector pivot, TSVector rightEdgeVertex) {
            rightPlane.set(pivot, pivot.Add(TSVector.up), rightEdgeVertex); // 高度
            rightPlane.normal = -rightPlane.normal; // 平面方向取反
            rightPlane.d = -rightPlane.d;
            rightPortal = rightEdgeVertex;
        }

        public void setPlanes(TSVector pivot, TriangleEdge edge) {
            setLeftPlane(pivot, edge.leftVertex);
            setRightPlane(pivot, edge.rightVertex);
        }

        public PlaneSide sideLeftPlane(TSVector point) {
            return leftPlane.testPoint(point);
        }

        public PlaneSide sideRightPlane(TSVector point) {
            return rightPlane.testPoint(point);
        }
    }
}