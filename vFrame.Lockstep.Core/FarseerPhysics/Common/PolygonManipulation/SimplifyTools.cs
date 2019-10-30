using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Common.PolygonManipulation
{
    public static class SimplifyTools
    {
        private static bool[] _usePt;
        private static FixedPoint _distanceTolerance;

        /// <summary>
        /// Removes all collinear points on the polygon.
        /// </summary>
        /// <param name="vertices">The polygon that needs simplification.</param>
        /// <param name="collinearityTolerance">The collinearity tolerance.</param>
        /// <returns>A simplified polygon.</returns>
        public static Vertices CollinearSimplify(Vertices vertices, FixedPoint collinearityTolerance) {
            //We can't simplify polygons under 3 vertices
            if (vertices.Count < 3)
                return vertices;

            Vertices simplified = new Vertices();

            for (int i = 0; i < vertices.Count; i++) {
                int prevId = vertices.PreviousIndex(i);
                int nextId = vertices.NextIndex(i);

                TSVector2 prev = vertices[prevId];
                TSVector2 current = vertices[i];
                TSVector2 next = vertices[nextId];

                //If they collinear, continue
                if (MathUtils.Collinear(ref prev, ref current, ref next, collinearityTolerance))
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Removes all collinear points on the polygon.
        /// Has a default bias of 0
        /// </summary>
        /// <param name="vertices">The polygon that needs simplification.</param>
        /// <returns>A simplified polygon.</returns>
        public static Vertices CollinearSimplify(Vertices vertices) {
            return CollinearSimplify(vertices, 0);
        }

        /// <summary>
        /// Ramer-Douglas-Peucker polygon simplification algorithm. This is the general recursive version that does not use the
        /// speed-up technique by using the Melkman convex hull.
        ///
        /// If you pass in 0, it will remove all collinear points
        /// </summary>
        /// <returns>The simplified polygon</returns>
        public static Vertices DouglasPeuckerSimplify(Vertices vertices, FixedPoint distanceTolerance) {
            _distanceTolerance = distanceTolerance;

            _usePt = new bool[vertices.Count];
            for (int i = 0; i < vertices.Count; i++)
                _usePt[i] = true;

            SimplifySection(vertices, 0, vertices.Count - 1);
            Vertices result = new Vertices();

            for (int i = 0; i < vertices.Count; i++)
                if (_usePt[i])
                    result.Add(vertices[i]);

            return result;
        }

        private static void SimplifySection(Vertices vertices, int i, int j) {
            if ((i + 1) == j)
                return;

            TSVector2 A = vertices[i];
            TSVector2 B = vertices[j];
            FixedPoint maxDistance = -1.0;
            int maxIndex = i;
            for (int k = i + 1; k < j; k++) {
                FixedPoint distance = DistancePointLine(vertices[k], A, B);

                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxIndex = k;
                }
            }

            if (maxDistance <= _distanceTolerance)
                for (int k = i + 1; k < j; k++)
                    _usePt[k] = false;
            else {
                SimplifySection(vertices, i, maxIndex);
                SimplifySection(vertices, maxIndex, j);
            }
        }

        private static FixedPoint DistancePointPoint(TSVector2 p, TSVector2 p2) {
            FixedPoint dx = p.x - p2.x;
            FixedPoint dy = p.y - p2.x;
            return TSMath.Sqrt(dx * dx + dy * dy);
        }

        private static FixedPoint DistancePointLine(TSVector2 p, TSVector2 A, TSVector2 B) {
            // if start == end, then use point-to-point distance
            if (A.x == B.x && A.y == B.y)
                return DistancePointPoint(p, A);

            // otherwise use comp.graphics.algorithms Frequently Asked Questions method
            /*(1)     	      AC dot AB
                        r =   ---------
                              ||AB||^2

		                r has the following meaning:
		                r=0 Point = A
		                r=1 Point = B
		                r<0 Point is on the backward extension of AB
		                r>1 Point is on the forward extension of AB
		                0<r<1 Point is interior to AB
	        */

            FixedPoint r = ((p.x - A.x) * (B.x - A.x) + (p.y - A.y) * (B.y - A.y))
                       /
                       ((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

            if (r <= 0.0) return DistancePointPoint(p, A);
            if (r >= 1.0) return DistancePointPoint(p, B);


            /*(2)
		                    (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
		                s = -----------------------------
		             	                Curve^2

		                Then the distance from C to Point = |s|*Curve.
	        */

            FixedPoint s = ((A.y - p.y) * (B.x - A.x) - (A.x - p.x) * (B.y - A.y))
                       /
                       ((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

            return TSMath.Abs(s) * TSMath.Sqrt(((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y)));
        }

        //From physics2d.net
        public static Vertices ReduceByArea(Vertices vertices, FixedPoint areaTolerance) {
            if (vertices.Count <= 3)
                return vertices;

            if (areaTolerance < 0) {
                throw new ArgumentOutOfRangeException("areaTolerance", "must be equal to or greater then zero.");
            }

            Vertices result = new Vertices();
            TSVector2 v1, v2, v3;
            FixedPoint old1, old2, new1;
            v1 = vertices[vertices.Count - 2];
            v2 = vertices[vertices.Count - 1];
            areaTolerance *= 2;
            for (int index = 0; index < vertices.Count; ++index, v2 = v3) {
                if (index == vertices.Count - 1) {
                    if (result.Count == 0) {
                        throw new ArgumentOutOfRangeException("areaTolerance", "The tolerance is too high!");
                    }

                    v3 = result[0];
                }
                else {
                    v3 = vertices[index];
                }

                MathUtils.Cross(ref v1, ref v2, out old1);
                MathUtils.Cross(ref v2, ref v3, out old2);
                MathUtils.Cross(ref v1, ref v3, out new1);
                if (TSMath.Abs(new1 - (old1 + old2)) > areaTolerance) {
                    result.Add(v2);
                    v1 = v2;
                }
            }

            return result;
        }

        //From Eric Jordan's convex decomposition library

        /// <summary>
        /// Merges all parallel edges in the list of vertices
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="tolerance">The tolerance.</param>
        public static void MergeParallelEdges(Vertices vertices, FixedPoint tolerance) {
            if (vertices.Count <= 3)
                return; //Can't do anything useful here to a triangle

            bool[] mergeMe = new bool[vertices.Count];
            int newNVertices = vertices.Count;

            //Gather points to process
            for (int i = 0; i < vertices.Count; ++i) {
                int lower = (i == 0) ? (vertices.Count - 1) : (i - 1);
                int middle = i;
                int upper = (i == vertices.Count - 1) ? (0) : (i + 1);

                FixedPoint dx0 = vertices[middle].x - vertices[lower].x;
                FixedPoint dy0 = vertices[middle].y - vertices[lower].y;
                FixedPoint dx1 = vertices[upper].y - vertices[middle].x;
                FixedPoint dy1 = vertices[upper].y - vertices[middle].y;
                FixedPoint norm0 = (FixedPoint) TSMath.Sqrt(dx0 * dx0 + dy0 * dy0);
                FixedPoint norm1 = (FixedPoint) TSMath.Sqrt(dx1 * dx1 + dy1 * dy1);

                if (!(norm0 > 0.0f && norm1 > 0.0f) && newNVertices > 3) {
                    //Merge identical points
                    mergeMe[i] = true;
                    --newNVertices;
                }

                dx0 /= norm0;
                dy0 /= norm0;
                dx1 /= norm1;
                dy1 /= norm1;
                FixedPoint cross = dx0 * dy1 - dx1 * dy0;
                FixedPoint dot = dx0 * dx1 + dy0 * dy1;

                if (TSMath.Abs(cross) < tolerance && dot > 0 && newNVertices > 3) {
                    mergeMe[i] = true;
                    --newNVertices;
                }
                else
                    mergeMe[i] = false;
            }

            if (newNVertices == vertices.Count || newNVertices == 0)
                return;

            int currIndex = 0;

            //Copy the vertices to a new list and clear the old
            Vertices oldVertices = new Vertices(vertices);
            vertices.Clear();

            for (int i = 0; i < oldVertices.Count; ++i) {
                if (mergeMe[i] || newNVertices == 0 || currIndex == newNVertices)
                    continue;

                Debug.Assert(currIndex < newNVertices);

                vertices.Add(oldVertices[i]);
                ++currIndex;
            }
        }

        //Misc

        /// <summary>
        /// Merges the identical points in the polygon.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <returns></returns>
        public static Vertices MergeIdenticalPoints(Vertices vertices) {
            //We use a dictonary here because HashSet is not avaliable on all platforms.
            HashSet<TSVector2> results = new HashSet<TSVector2>();

            for (int i = 0; i < vertices.Count; i++) {
                results.Add(vertices[i]);
            }

            Vertices returnResults = new Vertices();
            foreach (TSVector2 v in results) {
                returnResults.Add(v);
            }

            return returnResults;
        }

        /// <summary>
        /// Reduces the polygon by distance.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="distance">The distance between points. Points closer than this will be 'joined'.</param>
        /// <returns></returns>
        public static Vertices ReduceByDistance(Vertices vertices, FixedPoint distance) {
            //We can't simplify polygons under 3 vertices
            if (vertices.Count < 3)
                return vertices;

            Vertices simplified = new Vertices();

            for (int i = 0; i < vertices.Count; i++) {
                TSVector2 current = vertices[i];
                TSVector2 next = vertices.NextVertex(i);

                //If they are closer than the distance, continue
                if ((next - current).LengthSquared() <= distance)
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Reduces the polygon by removing the Nth vertex in the vertices list.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="nth">The Nth point to remove. Example: 5.</param>
        /// <returns></returns>
        public static Vertices ReduceByNth(Vertices vertices, int nth) {
            //We can't simplify polygons under 3 vertices
            if (vertices.Count < 3)
                return vertices;

            if (nth == 0)
                return vertices;

            Vertices result = new Vertices(vertices.Count);

            for (int i = 0; i < vertices.Count; i++) {
                if (i % nth == 0)
                    continue;

                result.Add(vertices[i]);
            }

            return result;
        }
    }
}