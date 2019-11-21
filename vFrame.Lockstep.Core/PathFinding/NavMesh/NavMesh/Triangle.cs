using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh
{
    public class Triangle
    {
        /** 三角形序号 */
        public int index;
        public TSVector a;
        public TSVector b;
        public TSVector c;

        public FixedPoint y; //三角形高度，三个顶点的平均高度

        /** 中点 */
        public TSVector center;

        /** 三角形和其他三角形的共享边 */
        public List<IConnection<Triangle>> connections;

        /**三角形顶点序号*/
        public int[] vectorIndex;

        public Triangle(TSVector a, TSVector b, TSVector c, int index, params int[] vectorIndex) {
            this.a = a;
            this.b = b;
            this.c = c;
            y = (a.y + b.y + c.y) / 3;
            this.index = index;
            center = a.Add(b).Add(c).scl(1 / new FixedPoint(3));
            connections = new List<IConnection<Triangle>>();
            this.vectorIndex = vectorIndex;
        }

        public override string ToString() {
            return "Triangle [index=" + index + ", a=" + a + ", b=" + b + ", c=" + c + ", center=" + center + "]";
        }

        public int getIndex() {
            return index;
        }

        public List<IConnection<Triangle>> getConnections() {
            return connections;
        }


        public bool IsInnerPoint(TSVector point) {
            var res = pointInLineLeft(a, b, point);
            if (res != pointInLineLeft(b, c, point)) return false;

            if (res != pointInLineLeft(c, a, point)) return false;

            if (cross2D(a, b, c) == 0) //三点共线
                return false;

            return true;
        }

        public static FixedPoint cross2D(TSVector fromPoint, TSVector toPoint, TSVector p) {
            return (toPoint.x - fromPoint.x) * (p.z - fromPoint.z) - (toPoint.z - fromPoint.z) * (p.x - fromPoint.x);
        }

        public static bool pointInLineLeft(TSVector fromPoint, TSVector toPoint, TSVector p) {
            return cross2D(fromPoint, toPoint, p) > 0;
        }


        public override int GetHashCode() {
            var prime = 31;
            var result = 1;
            result = prime * result + index;
            return result;
        }

        public override bool Equals(object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (GetType() != obj.GetType())
                return false;
            var other = (Triangle) obj;
            if (index != other.index)
                return false;
            return true;
        }
    }
}