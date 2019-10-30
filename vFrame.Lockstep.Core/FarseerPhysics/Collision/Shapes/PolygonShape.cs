/*
* Farseer Physics Engine based on Box2D.XNA port:
* Copyright (c) 2011 Ian Qvist
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
using vFrame.Lockstep.Core.FarseerPhysics.Common;

namespace vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes
{
    /// <summary>
    /// Represents a simple non-selfintersecting convex polygon.
    /// Create a convex hull from the given array of points.
    /// </summary>
    public class PolygonShape : Shape
    {
        public Vertices Normals;
        public Vertices Vertices;

        /// <summary>
        /// Initializes a new instance of the <see cref="PolygonShape"/> class.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="density">The density.</param>
        public PolygonShape(Vertices vertices, FixedPoint density)
            : base(density) {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;

            Set(vertices);
        }

        public PolygonShape(FixedPoint density)
            : base(density) {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            Normals = new Vertices(Settings.MaxPolygonVertices);
            Vertices = new Vertices(Settings.MaxPolygonVertices);
        }

        internal PolygonShape()
            : base(0) {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            Normals = new Vertices(Settings.MaxPolygonVertices);
            Vertices = new Vertices(Settings.MaxPolygonVertices);
        }

        public override int ChildCount {
            get { return 1; }
        }

        public override Shape Clone() {
            PolygonShape clone = new PolygonShape();
            clone.ShapeType = ShapeType;
            clone._radius = _radius;
            clone._density = _density;

            if (Settings.ConserveMemory) {
                clone.Vertices = Vertices;
                clone.Normals = Normals;
            }
            else {
                clone.Vertices = new Vertices(Vertices);
                clone.Normals = new Vertices(Normals);
            }

            clone.MassData = MassData;
            return clone;
        }

        /// <summary>
        /// Copy vertices. This assumes the vertices define a convex polygon.
        /// It is assumed that the exterior is the the right of each edge.
        /// </summary>
        /// @warning the points may be re-ordered, even if they form a convex polygon
        /// @warning collinear points are handled but not removed. Collinear points
        /// may lead to poor stacking behavior.
        /// <param name="input">The vertices.</param>
        public void Set(Vertices input) {
            Debug.Assert(input.Count >= 3 && input.Count <= Settings.MaxPolygonVertices);

            //TODO: Uncomment and remove the other line
            //Vertices = GiftWrap.GetConvexHull(input);
            Vertices = new Vertices(input);
            Normals = new Vertices(Vertices.Count);

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < Vertices.Count; ++i) {
                int i1 = i;
                int i2 = i + 1 < Vertices.Count ? i + 1 : 0;
                TSVector2 edge = Vertices[i2] - Vertices[i1];
                Debug.Assert(edge.LengthSquared() > Settings.Epsilon * Settings.Epsilon);

                TSVector2 temp = new TSVector2(edge.y, -edge.x);
                temp.Normalize();
                Normals.Add(temp);
            }

            // Compute the polygon mass data
            ComputeProperties();
        }

        /// <summary>
        /// Compute the mass properties of this shape using its dimensions and density.
        /// The inertia tensor is computed about the local origin, not the centroid.
        /// </summary>
        protected override void ComputeProperties() {
            // Polygon mass, centroid, and inertia.
            // Let rho be the polygon density in mass per unit area.
            // Then:
            // mass = rho * int(dA)
            // centroid.x = (1/mass) * rho * int(x * dA)
            // centroid.y = (1/mass) * rho * int(y * dA)
            // I = rho * int((x*x + y*y) * dA)
            //
            // We can compute these integrals by summing all the integrals
            // for each triangle of the polygon. To evaluate the integral
            // for a single triangle, we make a change of variables to
            // the (u,v) coordinates of the triangle:
            // x = x0 + e1x * u + e2x * v
            // y = y0 + e1y * u + e2y * v
            // where 0 <= u && 0 <= v && u + v <= 1.
            //
            // We integrate u from [0,1-v] and then v from [0,1].
            // We also need to use the Jacobian of the transformation:
            // D = cross(e1, e2)
            //
            // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
            //
            // The rest of the derivation is handled by computer algebra.

            Debug.Assert(Vertices.Count >= 3);

            if (_density <= 0)
                return;

            TSVector2 center = TSVector2.zero;
            FixedPoint area = 0.0f;
            FixedPoint I = 0.0f;

            // pRef is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            TSVector2 s = TSVector2.zero;

            // This code would put the reference point inside the polygon.
            for (int i = 0; i < Vertices.Count; ++i) {
                s += Vertices[i];
            }

            s *= 1.0f / Vertices.Count;

            FixedPoint k_inv3 = 1.0f / 3.0f;

            for (int i = 0; i < Vertices.Count; ++i) {
                // Triangle vertices.
                TSVector2 e1 = Vertices[i] - s;
                TSVector2 e2 = i + 1 < Vertices.Count ? Vertices[i + 1] - s : Vertices[0] - s;

                FixedPoint D = MathUtils.Cross(e1, e2);

                FixedPoint triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                center += triangleArea * k_inv3 * (e1 + e2);

                FixedPoint ex1 = e1.x, ey1 = e1.y;
                FixedPoint ex2 = e2.x, ey2 = e2.y;

                FixedPoint intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
                FixedPoint inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

                I += (0.25f * k_inv3 * D) * (intx2 + inty2);
            }

            //The area is too small for the engine to handle.
            Debug.Assert(area > Settings.Epsilon);

            // We save the area
            MassData.Area = area;

            // Total mass
            MassData.Mass = _density * area;

            // Center of mass
            center *= 1.0f / area;
            MassData.Centroid = center + s;

            // Inertia tensor relative to the local origin (point s).
            MassData.Inertia = _density * I;

            // Shift to center of mass then to original body origin.
            MassData.Inertia += MassData.Mass *
                                (TSVector2.Dot(MassData.Centroid, MassData.Centroid) - TSVector2.Dot(center, center));
        }

        /// <summary>
        /// Build vertices to represent an axis-aligned box.
        /// </summary>
        /// <param name="halfWidth">The half-width.</param>
        /// <param name="halfHeight">The half-height.</param>
        public void SetAsBox(FixedPoint halfWidth, FixedPoint halfHeight) {
            Set(PolygonTools.CreateRectangle(halfWidth, halfHeight));
        }

        /// <summary>
        /// Build vertices to represent an oriented box.
        /// </summary>
        /// <param name="halfWidth">The half-width..</param>
        /// <param name="halfHeight">The half-height.</param>
        /// <param name="center">The center of the box in local coordinates.</param>
        /// <param name="angle">The rotation of the box in local coordinates.</param>
        public void SetAsBox(FixedPoint halfWidth, FixedPoint halfHeight, TSVector2 center, FixedPoint angle) {
            Set(PolygonTools.CreateRectangle(halfWidth, halfHeight, center, angle));
        }

        /// <summary>
        /// Test a point for containment in this shape. This only works for convex shapes.
        /// </summary>
        /// <param name="transform">The shape world transform.</param>
        /// <param name="point">a point in world coordinates.</param>
        /// <returns>True if the point is inside the shape</returns>
        public override bool TestPoint(ref Transform transform, ref TSVector2 point) {
            TSVector2 pLocal = MathUtils.MulT(transform.q, point - transform.p);

            for (int i = 0; i < Vertices.Count; ++i) {
                FixedPoint dot = TSVector2.Dot(Normals[i], pLocal - Vertices[i]);
                if (dot > 0.0f) {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Cast a ray against a child shape.
        /// </summary>
        /// <param name="output">The ray-cast results.</param>
        /// <param name="input">The ray-cast input parameters.</param>
        /// <param name="transform">The transform to be applied to the shape.</param>
        /// <param name="childIndex">The child shape index.</param>
        /// <returns>True if the ray-cast hits the shape</returns>
        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform,
            int childIndex) {
            output = new RayCastOutput();

            // Put the ray into the polygon's frame of reference.
            TSVector2 p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            TSVector2 p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            TSVector2 d = p2 - p1;

            FixedPoint lower = 0.0f, upper = input.MaxFraction;

            int index = -1;

            for (int i = 0; i < Vertices.Count; ++i) {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                FixedPoint numerator = TSVector2.Dot(Normals[i], Vertices[i] - p1);
                FixedPoint denominator = TSVector2.Dot(Normals[i], d);

                if (denominator == 0.0f) {
                    if (numerator < 0.0f) {
                        return false;
                    }
                }
                else {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < 0.0f && numerator < lower * denominator) {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > 0.0f && numerator < upper * denominator) {
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

            Debug.Assert(0.0f <= lower && lower <= input.MaxFraction);

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

            return (Radius == shape.Radius &&
                    MassData == shape.MassData);
        }

        public override FixedPoint ComputeSubmergedArea(TSVector2 normal, FixedPoint offset, Transform xf, out TSVector2 sc) {
            sc = TSVector2.zero;

            //Transform plane into shape co-ordinates
            TSVector2 normalL = MathUtils.MulT(xf.q, normal);
            FixedPoint offsetL = offset - TSVector2.Dot(normal, xf.p);

            FixedPoint[] depths = new FixedPoint[Settings.MaxPolygonVertices];
            int diveCount = 0;
            int intoIndex = -1;
            int outoIndex = -1;

            bool lastSubmerged = false;
            int i;
            for (i = 0; i < Vertices.Count; i++) {
                depths[i] = TSVector2.Dot(normalL, Vertices[i]) - offsetL;
                bool isSubmerged = depths[i] < -Settings.Epsilon;
                if (i > 0) {
                    if (isSubmerged) {
                        if (!lastSubmerged) {
                            intoIndex = i - 1;
                            diveCount++;
                        }
                    }
                    else {
                        if (lastSubmerged) {
                            outoIndex = i - 1;
                            diveCount++;
                        }
                    }
                }

                lastSubmerged = isSubmerged;
            }

            switch (diveCount) {
                case 0:
                    if (lastSubmerged) {
                        //Completely submerged
                        sc = MathUtils.Mul(ref xf, MassData.Centroid);
                        return MassData.Mass / Density;
                    }
                    else {
                        //Completely dry
                        return 0;
                    }
                case 1:
                    if (intoIndex == -1) {
                        intoIndex = Vertices.Count - 1;
                    }
                    else {
                        outoIndex = Vertices.Count - 1;
                    }

                    break;
            }

            int intoIndex2 = (intoIndex + 1) % Vertices.Count;
            int outoIndex2 = (outoIndex + 1) % Vertices.Count;

            FixedPoint intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
            FixedPoint outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

            TSVector2 intoVec = new TSVector2(
                Vertices[intoIndex].x * (1 - intoLambda) + Vertices[intoIndex2].x * intoLambda,
                Vertices[intoIndex].y * (1 - intoLambda) + Vertices[intoIndex2].y * intoLambda);
            TSVector2 outoVec = new TSVector2(
                Vertices[outoIndex].x * (1 - outoLambda) + Vertices[outoIndex2].x * outoLambda,
                Vertices[outoIndex].y * (1 - outoLambda) + Vertices[outoIndex2].y * outoLambda);

            //Initialize accumulator
            FixedPoint area = 0;
            TSVector2 center = new TSVector2(0, 0);
            TSVector2 p2 = Vertices[intoIndex2];
            TSVector2 p3;

            FixedPoint k_inv3 = 1.0f / 3.0f;

            //An awkward loop from intoIndex2+1 to outIndex2
            i = intoIndex2;
            while (i != outoIndex2) {
                i = (i + 1) % Vertices.Count;
                if (i == outoIndex2)
                    p3 = outoVec;
                else
                    p3 = Vertices[i];
                //Add the triangle formed by intoVec,p2,p3
                {
                    TSVector2 e1 = p2 - intoVec;
                    TSVector2 e2 = p3 - intoVec;

                    FixedPoint D = MathUtils.Cross(e1, e2);

                    FixedPoint triangleArea = 0.5f * D;

                    area += triangleArea;

                    // Area weighted centroid
                    center += triangleArea * k_inv3 * (intoVec + p2 + p3);
                }
                //
                p2 = p3;
            }

            //Normalize and transform centroid
            center *= 1.0f / area;

            sc = MathUtils.Mul(ref xf, center);

            return area;
        }

        public bool Validate() {
            for (int i = 0; i < Vertices.Count; ++i) {
                int i1 = i;
                int i2 = i < Vertices.Count - 1 ? i1 + 1 : 0;
                TSVector2 p = Vertices[i1];
                TSVector2 e = Vertices[i2] - p;

                for (int j = 0; j < Vertices.Count; ++j) {
                    if (j == i1 || j == i2) {
                        continue;
                    }

                    TSVector2 v = Vertices[j] - p;
                    FixedPoint c = MathUtils.Cross(e, v);
                    if (c < 0.0f) {
                        return false;
                    }
                }
            }

            return true;
        }
    }
}