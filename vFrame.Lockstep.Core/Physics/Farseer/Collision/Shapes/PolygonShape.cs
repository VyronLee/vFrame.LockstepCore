/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// Represents a simple non-selfintersecting convex polygon.
    /// Create a convex hull from the given array of points.
    /// </summary>
    public class PolygonShape : Shape
    {
        internal Vertices _vertices;
        internal Vertices _normals;

        /// <summary>
        /// Initializes a new instance of the <see cref="PolygonShape"/> class.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="density">The density.</param>
        public PolygonShape(Vertices vertices)
            : base() {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;

            Vertices = vertices;
        }

        /// <summary>
        /// Create a new PolygonShape with the specified density.
        /// </summary>
        /// <param name="density">The density.</param>
        public PolygonShape(FixedPoint density)
            : base() {
            Debug.Assert(density >= FixedPoint.Zero);

            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            _vertices = new Vertices(Settings.MaxPolygonVertices);
            _normals = new Vertices(Settings.MaxPolygonVertices);
        }

        internal PolygonShape()
            : base() {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            _vertices = new Vertices(Settings.MaxPolygonVertices);
            _normals = new Vertices(Settings.MaxPolygonVertices);
        }

        /// <summary>
        /// Create a convex hull from the given array of local points.
        /// The number of vertices must be in the range [3, Settings.MaxPolygonVertices].
        /// Warning: the points may be re-ordered, even if they form a convex polygon
        /// Warning: collinear points are handled but not removed. Collinear points may lead to poor stacking behavior.
        /// </summary>
        public Vertices Vertices {
            get { return _vertices; }
            set {
                _vertices = new Vertices(value);

                Debug.Assert(_vertices.Count >= 3 && _vertices.Count <= Settings.MaxPolygonVertices);

                if (Settings.UseConvexHullPolygons) {
                    //FPE note: This check is required as the GiftWrap algorithm early exits on triangles
                    //So instead of giftwrapping a triangle, we just force it to be clock wise.
                    if (_vertices.Count <= 3)
                        _vertices.ForceCounterClockWise();
                    else
                        _vertices = GiftWrap.GetConvexHull(_vertices);
                }

                _vertices.ForceCounterClockWise();

                _normals = new Vertices(_vertices.Count);

                // Compute normals. Ensure the edges have non-zero length.
                for (int i = 0; i < _vertices.Count; ++i) {
                    int next = i + 1 < _vertices.Count ? i + 1 : 0;
                    TSVector2 edge = _vertices[next] - _vertices[i];
                    //Debug.Assert(edge.LengthSquared() > Settings.Epsilon * Settings.Epsilon);

                    //FPE optimization: Normals.Add(MathHelper.Cross(edge, FP.One));
                    TSVector2 temp = new TSVector2(edge.y, -edge.x);
                    temp.Normalize();
                    _normals.Add(temp);
                }
            }
        }

        public Vertices Normals {
            get { return _normals; }
        }

        public override int ChildCount {
            get { return 1; }
        }

        public override bool TestPoint(ref Transform transform, ref TSVector2 point) {
            TSVector2 pLocal = MathUtils.MulT(transform.q, point - transform.p);

            for (int i = 0; i < Vertices.Count; ++i) {
                FixedPoint dot = TSVector2.Dot(Normals[i], pLocal - Vertices[i]);
                if (dot > FixedPoint.Zero) {
                    return false;
                }
            }

            return true;
        }

        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform,
            int childIndex) {
            output = new RayCastOutput();

            // Put the ray into the polygon's frame of reference.
            TSVector2 p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            TSVector2 p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            TSVector2 d = p2 - p1;

            FixedPoint lower = FixedPoint.Zero, upper = input.MaxFraction;

            int index = -1;

            for (int i = 0; i < Vertices.Count; ++i) {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                FixedPoint numerator = TSVector2.Dot(Normals[i], Vertices[i] - p1);
                FixedPoint denominator = TSVector2.Dot(Normals[i], d);

                if (denominator == FixedPoint.Zero) {
                    if (numerator < FixedPoint.Zero) {
                        return false;
                    }
                }
                else {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < FixedPoint.Zero && numerator < lower * denominator) {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > FixedPoint.Zero && numerator < upper * denominator) {
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                // The use of epsilon here causes the assert on lower to trip
                // in some cases. Apparently the use of epsilon was to make edge
                // shapes work, but now those are handled separately.
                //if (upper < lower - b2_epsilon)
                if (upper < lower) {
                    return false;
                }
            }

            Debug.Assert(FixedPoint.Zero <= lower && lower <= input.MaxFraction);

            if (index >= 0) {
                output.Fraction = lower;
                output.Normal = MathUtils.Mul(transform.q, Normals[index]);
                return true;
            }

            return false;
        }

        /// <summary>
        /// Given a transform, compute the associated axis aligned bounding box for a child shape.
        /// </summary>
        /// <param name="aabb">The aabb results.</param>
        /// <param name="transform">The world transform of the shape.</param>
        /// <param name="childIndex">The child shape index.</param>
        public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex) {
            TSVector2 lower = MathUtils.Mul(ref transform, Vertices[0]);
            TSVector2 upper = lower;

            for (int i = 1; i < Vertices.Count; ++i) {
                TSVector2 v = MathUtils.Mul(ref transform, Vertices[i]);
                lower = TSVector2.Min(lower, v);
                upper = TSVector2.Max(upper, v);
            }

            TSVector2 r = new TSVector2(Radius, Radius);
            aabb.LowerBound = lower - r;
            aabb.UpperBound = upper + r;
        }

        public bool CompareTo(PolygonShape shape) {
            if (Vertices.Count != shape.Vertices.Count)
                return false;

            for (int i = 0; i < Vertices.Count; i++) {
                if (Vertices[i] != shape.Vertices[i])
                    return false;
            }

            return (Radius == shape.Radius);
        }

        public override Shape Clone() {
            PolygonShape clone = new PolygonShape();
            clone.ShapeType = ShapeType;
            clone._radius = _radius;
            clone._vertices = new Vertices(_vertices);
            clone._normals = new Vertices(_normals);
            return clone;
        }
    }
}