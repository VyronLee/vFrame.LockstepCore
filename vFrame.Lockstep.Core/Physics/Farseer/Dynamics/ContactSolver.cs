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

#pragma warning disable 0162

namespace vFrame.Lockstep.Core.Physics2D
{
    public static class WorldManifold
    {
        /// <summary>
        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the Shapes
        /// that generated the manifold.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="xfA">The transform for A.</param>
        /// <param name="radiusA">The radius for A.</param>
        /// <param name="xfB">The transform for B.</param>
        /// <param name="radiusB">The radius for B.</param>
        /// <param name="normal">World vector pointing from A to B</param>
        /// <param name="points">Torld contact point (point of intersection).</param>
        public static void Initialize(ref Manifold manifold, ref Transform xfA, FixedPoint radiusA, ref Transform xfB,
            FixedPoint radiusB, out TSVector2 normal, out FixedArray2<TSVector2> points) {
            normal = TSVector2.zero;
            points = new FixedArray2<TSVector2>();

            if (manifold.PointCount == 0) {
                return;
            }

            switch (manifold.Type) {
                case ManifoldType.Circles: {
                    normal = new TSVector2(FixedPoint.One, FixedPoint.Zero);
                    TSVector2 pointA = MathUtils.Mul(ref xfA, manifold.LocalPoint);
                    TSVector2 pointB = MathUtils.Mul(ref xfB, manifold.Points[0].LocalPoint);
                    if (TSVector2.DistanceSquared(pointA, pointB) > Settings.EpsilonSqr) {
                        normal = pointB - pointA;
                        normal.Normalize();
                    }

                    TSVector2 cA = pointA + radiusA * normal;
                    TSVector2 cB = pointB - radiusB * normal;
                    points[0] = FixedPoint.Half * (cA + cB);
                }
                    break;

                case ManifoldType.FaceA: {
                    normal = MathUtils.Mul(xfA.q, manifold.LocalNormal);
                    TSVector2 planePoint = MathUtils.Mul(ref xfA, manifold.LocalPoint);

                    for (int i = 0; i < manifold.PointCount; ++i) {
                        TSVector2 clipPoint = MathUtils.Mul(ref xfB, manifold.Points[i].LocalPoint);
                        TSVector2 cA = clipPoint + (radiusA - TSVector2.Dot(clipPoint - planePoint, normal)) * normal;
                        TSVector2 cB = clipPoint - radiusB * normal;
                        points[i] = FixedPoint.Half * (cA + cB);
                    }
                }
                    break;

                case ManifoldType.FaceB: {
                    normal = MathUtils.Mul(xfB.q, manifold.LocalNormal);
                    TSVector2 planePoint = MathUtils.Mul(ref xfB, manifold.LocalPoint);

                    for (int i = 0; i < manifold.PointCount; ++i) {
                        TSVector2 clipPoint = MathUtils.Mul(ref xfA, manifold.Points[i].LocalPoint);
                        TSVector2 cB = clipPoint + (radiusB - TSVector2.Dot(clipPoint - planePoint, normal)) * normal;
                        TSVector2 cA = clipPoint - radiusA * normal;
                        points[i] = FixedPoint.Half * (cA + cB);
                    }

                    // Ensure normal points from A to B.
                    normal = -normal;
                }
                    break;
            }
        }
    }
}