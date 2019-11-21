using System;
using System.Text;

/**
 * 相连接三角形的共享边
 *
 * @author JiangZhiYong
 * @QQ 359135103 2017年11月7日 下午4:50:11
 */

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh
{
    public class TriangleEdge : IConnection<Triangle>
    {
        /** 右顶点 */
        public TSVector rightVertex;
        public TSVector leftVertex;

        /** 源三角形 */
        public Triangle fromNode;

        /** 指向的三角形 */
        public Triangle toNode;

        public TriangleEdge(TSVector rightVertex, TSVector leftVertex) : this(null, null, rightVertex, leftVertex) {
        }

        public TriangleEdge(Triangle fromNode, Triangle toNode, TSVector rightVertex, TSVector leftVertex) {
            this.fromNode = fromNode;
            this.toNode = toNode;
            this.rightVertex = rightVertex;
            this.leftVertex = leftVertex;
        }

        public FixedPoint GetCost() {
            return FixedPoint.One;
        }

        public Triangle GetFromNode() {
            return fromNode;
        }

        public Triangle GetToNode() {
            return toNode;
        }

        public override string ToString() {
            var sb = new StringBuilder("Edge{");
            sb.Append("fromNode=").Append(fromNode.index);
            //sb.Append(", toNode=").Append(toNode == null ? "null" : toNode.index);
            sb.Append(", rightVertex=").Append(rightVertex);
            sb.Append(", leftVertex=").Append(leftVertex);
            sb.Append('}');
            return sb.ToString();
        }
    }
}