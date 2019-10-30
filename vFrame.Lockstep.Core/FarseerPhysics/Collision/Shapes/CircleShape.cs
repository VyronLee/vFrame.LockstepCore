/*
* Farseer Physics Engine:
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

using vFrame.Lockstep.Core.FarseerPhysics.Common;

namespace vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes
{
    public class CircleShape : Shape
    {
        internal TSVector2 _position;

        public CircleShape(FixedPoint radius, FixedPoint density)
            : base(density) {
            ShapeType = ShapeType.Circle;
            _radius = radius;
            _position = TSVector2.zero;
            ComputeProperties();
        }

        internal CircleShape()
            : base(0) {
            ShapeType = ShapeType.Circle;
            _radius = 0.0f;
            _position = TSVector2.zero;
        }

        public override int ChildCount {
            get { return 1; }
        }

        public TSVector2 Position {
            get { return _position; }
            set {
                _position = value;
                ComputeProperties();
            }
        }

        public override Shape Clone() {
            CircleShape shape = new CircleShape();
            shape._radius = Radius;
            shape._density = _density;
            shape._position = _position;
            shape.ShapeType = ShapeType;
            shape.MassData = MassData;
            return shape;
        }

        /// <summary>
        /// Test a point for containment in this shape. This only works for convex shapes.
        /// </summary>
        /// <param name="transform">The shape world transform.</param>
        /// <param name="point">a point in world coordinates.</param>
        /// <returns>True if the point is inside the shape</returns>
        public override bool TestPoint(ref Transform transform, ref TSVector2 point) {
            TSVector2 center = transform.p + MathUtils.Mul(transform.q, Position);
            TSVector2 d = point - center;
            return TSVector2.Dot(d, d) <= Radius * Radius;
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
            // Collision Detection in Interactive 3D Environments by Gino van den Bergen
            // From Section 3.1.2
            // x = s + a * r
            // norm(x) = radius

            output = new RayCastOutput();

            TSVector2 position = transform.p + MathUtils.Mul(transform.q, Position);
            TSVector2 s = input.Point1 - position;
            FixedPoint b = TSVector2.Dot(s, s) - Radius * Radius;

            // Solve quadratic equation.
            TSVector2 r = input.Point2 - input.Point1;
            FixedPoint c = TSVector2.Dot(s, r);
            FixedPoint rr = TSVector2.Dot(r, r);
            FixedPoint sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Settings.Epsilon) {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            FixedPoint a = -(c + (FixedPoint) TSMath.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.MaxFraction * rr) {
                a /= rr;
                output.Fraction = a;

                //TODO: Check results here
                output.Normal = s + a * r;
                output.Normal.Normalize();
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
            TSVector2 p = transform.p + MathUtils.Mul(transform.q, Position);
            aabb.LowerBound = new TSVector2(p.x - Radius, p.y - Radius);
            aabb.UpperBound = new TSVector2(p.x + Radius, p.y + Radius);
        }

        /// <summary>
        /// Compute the mass properties of this shape using its dimensions and density.
        /// The inertia tensor is computed about the local origin, not the centroid.
        /// </summary>
        protected override sealed void ComputeProperties() {
            FixedPoint area = Settings.Pi * Radius * Radius;
            MassData.Area = area;
            MassData.Mass = Density * area;
            MassData.Centroid = Position;

            // inertia about the local origin
            MassData.Inertia = MassData.Mass * (0.5f * Radius * Radius + TSVector2.Dot(Position, Position));
        }

        /// <summary>
        /// Compare the circle to another circle
        /// </summary>
        /// <param name="shape">The other circle</param>
        /// <returns>True if the two circles are the same size and have the same position</returns>
        public bool CompareTo(CircleShape shape) {
            return (Radius == shape.Radius && Position == shape.Position);
        }

        /// <summary>
        /// Method used by the BuoyancyController
        /// </summary>
        public override FixedPoint ComputeSubmergedArea(TSVector2 normal, FixedPoint offset, Transform xf, out TSVector2 sc) {
            sc = TSVector2.zero;

            TSVector2 p = MathUtils.Mul(ref xf, Position);
            FixedPoint l = -(TSVector2.Dot(normal, p) - offset);
            if (l < -Radius + Settings.Epsilon) {
                //Completely dry
                return 0;
            }

            if (l > Radius) {
                //Completely wet
                sc = p;
                return Settings.Pi * Radius * Radius;
            }

            //Magic
            FixedPoint r2 = Radius * Radius;
            FixedPoint l2 = l * l;
            FixedPoint area = r2 * (FixedPoint) ((TSMath.Asin(l / Radius) + Settings.Pi / 2) + l * TSMath.Sqrt(r2 - l2));
            FixedPoint com = -2.0f / 3.0f * (FixedPoint) TSMath.Pow(r2 - l2, 1.5f) / area;

            sc.x = p.x + normal.x * com;
            sc.y = p.y + normal.y * com;

            return area;
        }
    }
}