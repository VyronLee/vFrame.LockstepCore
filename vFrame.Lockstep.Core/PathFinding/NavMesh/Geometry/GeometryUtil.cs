using vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.Geometry
{
    public class GeometryUtil
    {
        public static FixedPoint FLOAT_ROUNDING_ERROR = FixedPoint.Epsilon;

        private static TSVector TMP_VEC_1 = new TSVector();
        private static TSVector TMP_VEC_2 = new TSVector();
        private static TSVector TMP_VEC_3 = new TSVector();

        /** Projects a point to a line segment. This implementation is thread-safe.	 */
        public static FixedPoint nearestSegmentPointSquareDistance
            (TSVector nearest, TSVector start, TSVector end, TSVector point) {
            nearest.set(start);
            var abX = end.x - start.x;
            var abY = end.y - start.y;
            var abZ = end.z - start.z;
            var abLen2 = abX * abX + abY * abY + abZ * abZ;
            if (abLen2 > 0) { // Avoid NaN due to the indeterminate form 0/0
                var t = ((point.x - start.x) * abX + (point.y - start.y) * abY + (point.z - start.z) * abZ) / abLen2;
                var s = TSMath.Clamp01(t);
                nearest.x += abX * s;
                nearest.y += abY * s;
                nearest.z += abZ * s;
            }

            return nearest.dst2(point);
        }


        /*
         * Find the closest point on the triangle, given a measure point.
         * This is the optimized algorithm taken from the book "Real-Time Collision Detection".
         * <p>
         * This implementation is NOT thread-safe.
         */
        public static FixedPoint getClosestPointOnTriangle(TSVector a, TSVector b, TSVector c, TSVector p,
            ref TSVector _out) {
            // Check if P in vertex region outside A
            var ab = b.sub(a);
            var ac = c.sub(a);
            var ap = p.sub(a);
            var d1 = ab.dot(ap);
            var d2 = ac.dot(ap);
            if (d1 <= 0 && d2 <= 0) {
                _out = a;
                return p.dst2(a);
            }

            // Check if P in vertex region outside B
            var bp = p.sub(b);
            var d3 = ab.dot(bp);
            var d4 = ac.dot(bp);
            if (d3 >= 0 && d4 <= d3) {
                _out = b;
                return p.dst2(b);
            }

            // Check if P in edge region of AB, if so return projection of P onto AB
            var vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0) {
                var v = d1 / (d1 - d3);
                _out.set(a).mulAdd(ab, v); // barycentric coordinates (1-v,v,0)
                return p.dst2(_out);
            }

            // Check if P in vertex region outside C
            var cp = p.sub(c);
            var d5 = ab.dot(cp);
            var d6 = ac.dot(cp);
            if (d6 >= 0 && d5 <= d6) {
                _out = c;
                return p.dst2(c);
            }

            // Check if P in edge region of AC, if so return projection of P onto AC
            var vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0) {
                var w = d2 / (d2 - d6);
                _out.set(a).mulAdd(ac, w); // barycentric coordinates (1-w,0,w)
                return _out.dst2(p);
            }

            // Check if P in edge region of BC, if so return projection of P onto BC
            var va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
                var w = (d4 - d3) / (d4 - d3 + (d5 - d6));
                _out.set(b).mulAdd(c.sub(b), w); // barycentric coordinates (0,1-w,w)
                return _out.dst2(p);
            }

            // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
            var denom = 1 / (va + vb + vc);
            {
                var v = vb * denom;
                var w = vc * denom;
                _out.set(a).mulAdd(ab, v).mulAdd(ac, w);
            }
            return _out.dst2(p);
        }

        public static bool IntersectRayTriangle(Ray ray, TSVector t1, TSVector t2, TSVector t3,
            out TSVector intersection) {
            intersection = TSVector.zero;
            var edge1 = t2.sub(t1);
            var edge2 = t3.sub(t1);

            var pvec = ray.direction.cross(edge2);
            var det = edge1.dot(pvec);
            if (IsZero(det)) {
                var p = new Plane(t1, t2, t3);
                if (p.testPoint(ray.origin) == PlaneSide.OnPlane && IsPointInTriangle(ray.origin, t1, t2, t3)) {
                    intersection.set(ray.origin);
                    return true;
                }

                return false;
            }

            det = 1 / det;

            var tvec = ray.origin.sub(t1);
            var u = tvec.dot(pvec) * det;
            if (u < 0 || u > 1)
                return false;

            var qvec = tvec.cross(edge1);
            var v = ray.direction.dot(qvec) * det;
            if (v < 0 || u + v > 1)
                return false;

            var t = edge2.dot(qvec) * det;
            if (t < 0)
                return false;

            if (t <= FLOAT_ROUNDING_ERROR)
                intersection.set(ray.origin);
            else
                ray.getEndPoint(intersection, t);

            return true;
        }

        public static bool IsPointInTriangle(TSVector point, TSVector t1, TSVector t2, TSVector t3) {
            var v0 = t1.sub(point);
            var v1 = t2.sub(point);
            var v2 = t3.sub(point);

            var ab = v0.dot(v1);
            var ac = v0.dot(v2);
            var bc = v1.dot(v2);
            var cc = v2.dot(v2);

            if (bc * ac - cc * ab < 0)
                return false;
            var bb = v1.dot(v1);
            if (ab * bc - ac * bb < 0)
                return false;
            return true;
        }

        public static bool IsZero(FixedPoint value) {
            return TSMath.Abs(value) <= FLOAT_ROUNDING_ERROR;
        }
    }
}