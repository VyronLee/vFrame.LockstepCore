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

using System;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        internal TSVector2 _position;

        /// <summary>
        /// Create a new circle with the desired radius and density.
        /// </summary>
        /// <param name="radius">The radius of the circle.</param>
        /// <param name="density">The density of the circle.</param>
        public CircleShape(FP radius)
            : base()
        {
            Debug.Assert(radius >= 0);

            ShapeType = ShapeType.Circle;
            _position = TSVector2.zero;
            Radius = radius; // The Radius property cache 2radius and calls ComputeProperties(). So no need to call ComputeProperties() here.
        }

        internal CircleShape()
            : base()
        {
            ShapeType = ShapeType.Circle;
            _radius = FP.Zero;
            _position = TSVector2.zero;
        }

        public override int ChildCount
        {
            get { return 1; }
        }

        /// <summary>
        /// Get or set the position of the circle
        /// </summary>
        public TSVector2 Position
        {
            get { return _position; }
            set
            {
                _position = value;
            }
        }

        public override bool TestPoint(ref Transform transform, ref TSVector2 point)
        {
            TSVector2 center = transform.p + MathUtils.Mul(transform.q, Position);
            TSVector2 d = point - center;
            return TSVector2.Dot(d, d) <= _2radius;
        }

        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex)
        {
            // Collision Detection in Interactive 3D Environments by Gino van den Bergen
            // From Section 3.1.2
            // x = s + a * r
            // norm(x) = radius

            output = new RayCastOutput();

            TSVector2 position = transform.p + MathUtils.Mul(transform.q, Position);
            TSVector2 s = input.Point1 - position;
            FP b = TSVector2.Dot(s, s) - _2radius;

            // Solve quadratic equation.
            TSVector2 r = input.Point2 - input.Point1;
            FP c = TSVector2.Dot(s, r);
            FP rr = TSVector2.Dot(r, r);
            FP sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < FP.Zero || rr < Settings.Epsilon)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            FP a = -(c + FP.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (FP.Zero <= a && a <= input.MaxFraction * rr)
            {
                a /= rr;
                output.Fraction = a;

                //TODO: Check results here
                output.Normal = s + a * r;
                output.Normal.Normalize();
                return true;
            }

            return false;
        }

        public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
        {
            TSVector2 p = transform.p + MathUtils.Mul(transform.q, Position);
            aabb.LowerBound = new TSVector2(p.x - Radius, p.y - Radius);
            aabb.UpperBound = new TSVector2(p.x + Radius, p.y + Radius);
        }

        /// <summary>
        /// Compare the circle to another circle
        /// </summary>
        /// <param name="shape">The other circle</param>
        /// <returns>True if the two circles are the same size and have the same position</returns>
        public bool CompareTo(CircleShape shape)
        {
            return (Radius == shape.Radius && Position == shape.Position);
        }

        public override Shape Clone()
        {
            CircleShape clone = new CircleShape();
            clone.ShapeType = ShapeType;
            clone._radius = Radius;
            clone._2radius = _2radius; //FPE note: We also copy the cache
            clone._position = _position;
            return clone;
        }
    }
}