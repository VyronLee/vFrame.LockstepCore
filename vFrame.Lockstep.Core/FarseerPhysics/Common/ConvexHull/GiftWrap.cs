namespace vFrame.Lockstep.Core.FarseerPhysics.Common.ConvexHull
{
    public static class GiftWrap
    {
        //Extracted from Box2D

        /// <summary>
        /// Giftwrap convex hull algorithm
        /// O(nh) time complexity, where n is the number of points and h is the number of points on the convex hull.
        /// See http://en.wikipedia.org/wiki/Gift_wrapping_algorithm for more details.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <returns></returns>
        public static Vertices GetConvexHull(Vertices vertices) {
            // Find the right most point on the hull
            int i0 = 0;
            FixedPoint x0 = vertices[0].x;
            for (int i = 1; i < vertices.Count; ++i) {
                FixedPoint x = vertices[i].x;
                if (x > x0 || (x == x0 && vertices[i].y < vertices[i0].y)) {
                    i0 = i;
                    x0 = x;
                }
            }

            int[] hull = new int[vertices.Count];
            int m = 0;
            int ih = i0;

            for (;;) {
                hull[m] = ih;

                int ie = 0;
                for (int j = 1; j < vertices.Count; ++j) {
                    if (ie == ih) {
                        ie = j;
                        continue;
                    }

                    TSVector2 r = vertices[ie] - vertices[hull[m]];
                    TSVector2 v = vertices[j] - vertices[hull[m]];
                    FixedPoint c = MathUtils.Cross(r, v);
                    if (c < 0.0f) {
                        ie = j;
                    }

                    // Collinearity check
                    if (c == 0.0f && v.LengthSquared() > r.LengthSquared()) {
                        ie = j;
                    }
                }

                ++m;
                ih = ie;

                if (ie == i0) {
                    break;
                }
            }

            Vertices result = new Vertices();

            // Copy vertices.
            for (int i = 0; i < m; ++i) {
                result.Add(vertices[hull[i]]);
            }

            return result;
        }
    }
}