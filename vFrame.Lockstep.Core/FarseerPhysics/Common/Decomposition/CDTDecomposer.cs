/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using System.Collections.Generic;
using vFrame.Lockstep.Core.FarseerPhysics.Common.Decomposition.CDT;
using vFrame.Lockstep.Core.FarseerPhysics.Common.Decomposition.CDT.Delaunay;
using vFrame.Lockstep.Core.FarseerPhysics.Common.Decomposition.CDT.Delaunay.Sweep;
using vFrame.Lockstep.Core.FarseerPhysics.Common.Decomposition.CDT.Polygon;
using vFrame.Lockstep.Core.FarseerPhysics.Common.TextureTools;

namespace vFrame.Lockstep.Core.FarseerPhysics.Common.Decomposition
{
    public static class CDTDecomposer
    {
        public static List<Vertices> ConvexPartition(Vertices vertices) {
            Polygon poly = new Polygon();

            foreach (TSVector2 vertex in vertices) {
                poly.Points.Add(new TriangulationPoint(vertex.x, vertex.y));
            }

            DTSweepContext tcx = new DTSweepContext();
            tcx.PrepareTriangulation(poly);
            DTSweep.Triangulate(tcx);

            List<Vertices> results = new List<Vertices>();

            foreach (DelaunayTriangle triangle in poly.Triangles) {
                Vertices v = new Vertices();
                foreach (TriangulationPoint p in triangle.Points) {
                    v.Add(new TSVector2((FixedPoint) p.X, (FixedPoint) p.Y));
                }

                results.Add(v);
            }

            return results;
        }

        public static List<Vertices> ConvexPartition(DetectedVertices vertices) {
            Polygon poly = new Polygon();
            foreach (var vertex in vertices)
                poly.Points.Add(new TriangulationPoint(vertex.x, vertex.y));

            if (vertices.Holes != null) {
                foreach (var holeVertices in vertices.Holes) {
                    Polygon hole = new Polygon();
                    foreach (var vertex in holeVertices)
                        hole.Points.Add(new TriangulationPoint(vertex.x, vertex.y));

                    poly.AddHole(hole);
                }
            }

            DTSweepContext tcx = new DTSweepContext();
            tcx.PrepareTriangulation(poly);
            DTSweep.Triangulate(tcx);

            List<Vertices> results = new List<Vertices>();

            foreach (DelaunayTriangle triangle in poly.Triangles) {
                Vertices v = new Vertices();
                foreach (TriangulationPoint p in triangle.Points) {
                    v.Add(new TSVector2((FixedPoint) p.X, (FixedPoint) p.Y));
                }

                results.Add(v);
            }

            return results;
        }

        public static List<Vertices> ConvexPartition(List<DetectedVertices> vertices) {
            List<Vertices> result = new List<Vertices>();

            foreach (var e in vertices)
                result.AddRange(ConvexPartition(e));

            return result;
        }
    }
}