using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh
{
    internal static class VectorExtension
    {
        public static TSVector set(this TSVector vec, FixedPoint x, FixedPoint y, FixedPoint z) {
            vec.x = x;
            vec.y = y;
            vec.z = z;
            return vec;
        }

        public static TSVector set(this TSVector vec, TSVector val) {
            vec = val;
            return vec;
        }

        public static TSVector mulAdd(this TSVector _this, TSVector vec, FixedPoint scalar) {
            _this.x += vec.x * scalar;
            _this.y += vec.y * scalar;
            _this.z += vec.z * scalar;
            return _this;
        }

        public static TSVector Add(this TSVector vec, TSVector val) {
            return vec + val;
        }

        public static TSVector sub(this TSVector vec, TSVector val) {
            return vec - val;
        }

        public static TSVector scl(this TSVector vec, FixedPoint val) {
            return vec * val;
        }

        public static FixedPoint dot(this TSVector vec, TSVector val) {
            return TSVector.Dot(vec, val);
        }

        public static FixedPoint dot(this TSVector vec, FixedPoint x, FixedPoint y, FixedPoint z) {
            return TSVector.Dot(vec, new TSVector(x, y, z));
        }


        public static TSVector cross(this TSVector vec, TSVector vector) {
            return new TSVector(vec.y * vector.z - vec.z * vector.y, vec.z * vector.x - vec.x * vector.z,
                vec.x * vector.y - vec.y * vector.x);
        }

        public static TSVector cross(this TSVector vec, FixedPoint x, FixedPoint y, FixedPoint z) {
            return new TSVector(vec.y * z - vec.z * y, vec.z * x - vec.x * z, vec.x * y - vec.y * x);
        }

        public static TSVector nor(this TSVector vec) {
            return vec.normalized;
        }

        public static FixedPoint len(this TSVector vec) {
            return vec.magnitude;
        }

        public static FixedPoint dst2(this TSVector vec, TSVector p) {
            return dst2(vec.x, vec.z, p.x, p.z);
        }

        public static FixedPoint dst2(FixedPoint x1, FixedPoint z1, FixedPoint x2, FixedPoint z2) {
            x1 -= x2;
            z1 -= z2;
            return x1 * x1 + z1 * z1;
        }

        public static T get<T>(this List<T> lst, int idx) {
            return lst[idx];
        }

        public static Tval get<Tkey, Tval>(this Dictionary<Tkey, Tval> lst, Tkey idx) {
            return lst[idx];
        }
    }
}