﻿using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// Convex decomposition algorithm created by unknown
    /// 
    /// Properties:
    /// - No support for holes
    /// - Very fast
    /// - Only works on simple polygons
    /// - Only works on counter clockwise polygons
    /// 
    /// More information: http://www.flipcode.com/archives/Efficient_Polygon_Triangulation.shtml
    /// </summary>
    internal static class FlipcodeDecomposer
    {
        private static TSVector2 _tmpA;
        private static TSVector2 _tmpB;
        private static TSVector2 _tmpC;

        /// <summary>
        /// Decompose the polygon into triangles.
        /// 
        /// Properties:
        /// - Only works on counter clockwise polygons
        /// 
        /// </summary>
        /// <param name="vertices">The list of points describing the polygon</param>
        public static List<Vertices> ConvexPartition(Vertices vertices)
        {
            Debug.Assert(vertices.Count > 3);
            Debug.Assert(vertices.IsCounterClockWise());

            int[] polygon = new int[vertices.Count];

            for (int v = 0; v < vertices.Count; v++)
                polygon[v] = v;

            int nv = vertices.Count;

            // Remove nv-2 Vertices, creating 1 triangle every time
            int count = 2 * nv; /* error detection */

            List<Vertices> result = new List<Vertices>();

            for (int v = nv - 1; nv > 2; )
            {
                // If we loop, it is probably a non-simple polygon 
                if (0 >= (count--))
                {
                    // Triangulate: ERROR - probable bad polygon!
                    return new List<Vertices>();
                }

                // Three consecutive vertices in current polygon, <u,v,w>
                int u = v;
                if (nv <= u)
                    u = 0; // Previous 
                v = u + 1;
                if (nv <= v)
                    v = 0; // New v   
                int w = v + 1;
                if (nv <= w)
                    w = 0; // Next 

                _tmpA = vertices[polygon[u]];
                _tmpB = vertices[polygon[v]];
                _tmpC = vertices[polygon[w]];

                if (Snip(vertices, u, v, w, nv, polygon))
                {
                    int s, t;

                    // Output Triangle
                    Vertices triangle = new Vertices(3);
                    triangle.Add(_tmpA);
                    triangle.Add(_tmpB);
                    triangle.Add(_tmpC);
                    result.Add(triangle);

                    // Remove v from remaining polygon 
                    for (s = v, t = v + 1; t < nv; s++, t++)
                    {
                        polygon[s] = polygon[t];
                    }
                    nv--;

                    // Reset error detection counter
                    count = 2 * nv;
                }
            }

            return result;
        }

        /// <summary>
        /// Check if the point P is inside the triangle defined by
        /// the points A, B, C
        /// </summary>
        /// <param name="a">The A point.</param>
        /// <param name="b">The B point.</param>
        /// <param name="c">The C point.</param>
        /// <param name="p">The point to be tested.</param>
        /// <returns>True if the point is inside the triangle</returns>
        private static bool InsideTriangle(ref TSVector2 a, ref TSVector2 b, ref TSVector2 c, ref TSVector2 p)
        {
            //A cross bp
            FP abp = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);

            //A cross ap
            FP aap = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);

            //b cross cp
            FP bcp = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

            return ((abp >= FP.Zero) && (bcp >= FP.Zero) && (aap >= FP.Zero));
        }

        /// <summary>
        /// Cut a the contour and add a triangle into V to describe the 
        /// location of the cut
        /// </summary>
        /// <param name="contour">The list of points defining the polygon</param>
        /// <param name="u">The index of the first point</param>
        /// <param name="v">The index of the second point</param>
        /// <param name="w">The index of the third point</param>
        /// <param name="n">The number of elements in the array.</param>
        /// <param name="V">The array to populate with indicies of triangles.</param>
        /// <returns>True if a triangle was found</returns>
        private static bool Snip(Vertices contour, int u, int v, int w, int n, int[] V)
        {
            if (Settings.Epsilon > MathUtils.Area(ref _tmpA, ref _tmpB, ref _tmpC))
                return false;

            for (int p = 0; p < n; p++)
            {
                if ((p == u) || (p == v) || (p == w))
                    continue;

                TSVector2 point = contour[V[p]];

                if (InsideTriangle(ref _tmpA, ref _tmpB, ref _tmpC, ref point))
                    return false;
            }

            return true;
        }
    }
}