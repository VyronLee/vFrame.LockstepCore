using System;

namespace vFrame.Lockstep.Core.PathFinding
{
    public class Plane
    {
        private static long serialVersionUID = -1240652082930747866L;

        public TSVector normal = new TSVector(); // 单位长度
        public FixedPoint d = FixedPoint.Zero; // 距离

        public Plane() {
        }

        public Plane(TSVector normal, FixedPoint d) {
            this.normal.set(normal).nor();
            this.d = d;
        }

        public Plane(TSVector normal, TSVector point) {
            this.normal.set(normal).nor();
            this.d = -this.normal.dot(point);
        }

        public Plane(TSVector point1, TSVector point2, TSVector point3) {
            set(point1, point2, point3);
        }

        public void set(TSVector point1, TSVector point2, TSVector point3) {
            normal = (point1).sub(point2).cross(point2.x - point3.x, point2.y - point3.y, point2.z - point3.z).nor();
            d = -point1.dot(normal);
        }


        public FixedPoint distance(TSVector point) {
            return normal.dot(point) + d;
        }

        public PlaneSide testPoint(TSVector point) {
            FixedPoint dist = normal.dot(point) + d;

            if (dist == 0)
                return PlaneSide.OnPlane;
            else if (dist < 0)
                return PlaneSide.Back;
            else
                return PlaneSide.Front;
        }


        public PlaneSide testPoint(FixedPoint x, FixedPoint y, FixedPoint z) {
            FixedPoint dist = normal.dot(x, y, z) + d;

            if (dist == 0)
                return PlaneSide.OnPlane;
            else if (dist < 0)
                return PlaneSide.Back;
            else
                return PlaneSide.Front;
        }


        public bool isFrontFacing(TSVector direction) {
            FixedPoint dot = normal.dot(direction);
            return dot <= 0;
        }

        /** @return The normal */
        public TSVector getNormal() {
            return normal;
        }

        /** @return The distance to the origin */
        public FixedPoint getD() {
            return d;
        }


        public void set(TSVector point, TSVector normal) {
            this.normal.set(normal);
            d = -point.dot(normal);
        }

        public void set(FixedPoint pointX, FixedPoint pointY, FixedPoint pointZ, FixedPoint norX, FixedPoint norY,
            FixedPoint norZ) {
            this.normal.set(norX, norY, norZ);
            d = -(pointX * norX + pointY * norY + pointZ * norZ);
        }


        public void set(Plane plane) {
            this.normal.set(plane.normal);
            this.d = plane.d;
        }

        public override String ToString() {
            return normal.ToString() + ", " + d;
        }
    }
}