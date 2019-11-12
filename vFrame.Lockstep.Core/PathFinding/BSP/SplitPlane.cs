namespace vFrame.Lockstep.Core.PathFinding
{
    public struct SplitPlane
    {
        public SplitPlane(TSVector2 a, TSVector2 b) {
            this.a = a;
            this.b = b;
        }

        public TSVector2 a;
        public TSVector2 b;
        public TSVector2 dir => b - a;

        private static FixedPoint val;

        public static ESplitType GetSplitResult(SplitPlane plane, TriRef tri) {
            var planeDir = plane.dir;
            var valA = TSVector2.Cross(planeDir, tri.a - plane.a);
            var valB = TSVector2.Cross(planeDir, tri.b - plane.a);
            var valC = TSVector2.Cross(planeDir, tri.c - plane.a);

            var isRight = false;
            if (valA != 0) isRight = valA < 0;
            if (valB != 0) isRight = valB < 0;
            if (valC != 0) isRight = valC < 0;

            var isA = valA <= 0;
            var isB = valB <= 0;
            var isC = valC <= 0;
            if (isA == isB && isB == isC) {
                return isRight ? ESplitType.Right : ESplitType.Left;
            }

            isA = valA >= 0;
            isB = valB >= 0;
            isC = valC >= 0;
            if (isA == isB && isB == isC) {
                return isRight ? ESplitType.Right : ESplitType.Left;
            }

            return ESplitType.OnPlane;
        }

        public static ESplitType ClassifyPointToPlane(SplitPlane plane, TSVector2 vertex) {
            var val = TSVector2.Cross(plane.dir, vertex - plane.a);
            if (val == 0)
                return ESplitType.OnPlane;
            else {
                return val < 0 ? ESplitType.Right : ESplitType.Left;
            }
        }

        public static TSVector2 GetIntersectPoint(TSVector2 p0, TSVector2 p1, TSVector2 p2, TSVector2 p3) {
            var diff = p2 - p0;
            var d1 = p1 - p0;
            var d2 = p3 - p2;
            var demo = TSMath.Cross2D(d1, d2); //det
            if (TSMath.Abs(demo) < FixedPoint.Epsilon) //parallel
                return p0;

            var t1 = TSMath.Cross2D(diff, d2) / demo; // Cross2D(diff,-d2)
            return p0 + (p1 - p0) * t1;
        }
    }
}