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
//#define USE_ACTIVE_CONTACT_SET
//#define USE_AWAKE_BODY_SET
//#define USE_ISLAND_SET
//#define OPTIMIZE_TOI
//#define USE_IGNORE_CCD_CATEGORIES

using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// The world class manages all physics entities, dynamic simulation,
    /// and asynchronous queries.
    /// </summary>
    public class World : IWorld
    {
        internal FixedPoint _invDt0 = 0f;
        internal Body[] _stack = new Body[64];
        internal bool _stepComplete = false;
        internal List<Body> _bodyAddList = new List<Body>();
        internal List<Body> _bodyRemoveList = new List<Body>();

        internal Func<Fixture, bool> _queryAABBCallback;
        private Func<int, bool> _queryAABBCallbackWrapper;
        internal TOIInput _input = new TOIInput();
        internal Fixture _myFixture;
        private TSVector2 _point1;
        internal TSVector2 _point2;
        internal List<Fixture> _testPointAllFixtures;
        private Func<Fixture, TSVector2, TSVector2, FixedPoint, FixedPoint> _rayCastCallback;
        private Func<RayCastInput, int, int, FixedPoint> _rayCastCallbackWrapper;
        private Func<CircleCastInput, int, int, FixedPoint> _circleCastCallbackWrapper;

        internal Queue<Contact> _contactPool = new Queue<Contact>(256);
        internal bool _worldHasNewFixture;

        /// <summary>
        /// Fires whenever a body has been added
        /// </summary>
        public BodyDelegate BodyAdded;

        /// <summary>
        /// Fires whenever a body has been removed
        /// </summary>
        public BodyDelegate BodyRemoved;

        /// <summary>
        /// Fires whenever a fixture has been added
        /// </summary>
        public FixtureDelegate FixtureAdded;

        /// <summary>
        /// Fires whenever a fixture has been removed
        /// </summary>
        public FixtureDelegate FixtureRemoved;

        public IBroadPhase BroadPhase;

        /// <summary>
        /// Initializes a new instance of the <see cref="World"/> class.
        /// </summary>
        public World(TSVector2 gravity) {
            Enabled = true;
            BodyList = new List<Body>(32);

#if USE_AWAKE_BODY_SET
            AwakeBodySet = new HashSet<Body>();
            AwakeBodyList = new List<Body>(32);
#endif
#if USE_ISLAND_SET
            IslandSet = new HashSet<Body>();
#endif
#if OPTIMIZE_TOI
            TOISet = new HashSet<Body>();
#endif

            _queryAABBCallbackWrapper = QueryAABBCallbackWrapper;
            _rayCastCallbackWrapper = RayCastCallbackWrapper;
            _circleCastCallbackWrapper = CircleCastCallbackWrapper;

            BroadPhase = new DynamicTreeBroadPhase();
            Gravity = gravity;
        }

        private void ProcessRemovedJoints() {
        }

        private void ProcessAddedJoints() {
        }

        public void ProcessAddedBodies() {
            if (_bodyAddList.Count > 0) {
                foreach (Body body in _bodyAddList) {
#if USE_AWAKE_BODY_SET
                    Debug.Assert(!body.IsDisposed);
                    if (body.Awake)
                    {
                        if (!AwakeBodySet.Contains(body))
                        {
                            AwakeBodySet.Add(body);
                        }
                    }
                    else
                    {
                        if (AwakeBodySet.Contains(body))
                        {
                            AwakeBodySet.Remove(body);
                        }
                    }
#endif
                    // Add to world list.
                    BodyList.Add(body);

                    if (BodyAdded != null)
                        BodyAdded(body);
                }

                _bodyAddList.Clear();
            }
        }

        public void ProcessRemovedBodies() {
            if (_bodyRemoveList.Count > 0) {
                foreach (Body body in _bodyRemoveList) {
                    Debug.Assert(BodyList.Count > 0);

                    // You tried to remove a body that is not contained in the BodyList.
                    // Are you removing the body more than once?
                    //Debug.Assert(BodyList.Contains(body));

                    bool isOnAddList = _bodyAddList.Contains(body);

                    if (!BodyList.Contains(body) && !isOnAddList) {
                        continue;
                    }

#if USE_AWAKE_BODY_SET
                    Debug.Assert(!AwakeBodySet.Contains(body));
#endif

                    // Delete the attached fixtures. This destroys broad-phase proxies.
                    for (int i = 0; i < body.FixtureList.Count; i++) {
                        body.FixtureList[i].DestroyProxies(BroadPhase);
                        body.FixtureList[i].Destroy();
                    }

                    body.FixtureList = null;

                    if (isOnAddList) {
                        _bodyAddList.Remove(body);
                    }
                    else {
                        BodyList.Remove(body);
                    }

                    if (BodyRemoved != null && body._specialSensor == BodySpecialSensor.None)
                        BodyRemoved(body);

#if USE_AWAKE_BODY_SET
                    Debug.Assert(!AwakeBodySet.Contains(body));
#endif
                }

                _bodyRemoveList.Clear();
            }
        }

        private bool QueryAABBCallbackWrapper(int proxyId) {
            FixtureProxy proxy = BroadPhase.GetProxy(proxyId);
            return _queryAABBCallback(proxy.Fixture);
        }

        private FixedPoint RayCastCallbackWrapper(RayCastInput rayCastInput, int proxyId, int layerMask) {
            FixtureProxy proxy = BroadPhase.GetProxy(proxyId);
            Fixture fixture = proxy.Fixture;
            if (((int) fixture.CollisionCategories & layerMask) == 0) {
                return rayCastInput.MaxFraction;
            }

            int index = proxy.ChildIndex;
            RayCastOutput output;
            bool hit = fixture.RayCast(out output, ref rayCastInput, index);

            if (hit) {
                FixedPoint fraction = output.Fraction;
                TSVector2 point = (FixedPoint.One - fraction) * rayCastInput.Point1 + fraction * rayCastInput.Point2;
                return _rayCastCallback(fixture, point, output.Normal, fraction);
            }

            return rayCastInput.MaxFraction;
        }

        private bool ConvertHitPointAndNormal(Transform transA, Transform transB, Fixture fixtureA, Fixture fixtureB,
            int shapeIndex,
            FixedPoint shapeRadiusA, FixedPoint shapeRadiusB,
            out TSVector2 hitPoint,
            out TSVector2 hitNoraml,
            out bool zeroNormal) {
            var contact = Contact.Create(this, fixtureA, 0, fixtureB, shapeIndex);
            var NotSwapped = contact.FixtureA == fixtureA;

            zeroNormal = false;

            if (!NotSwapped) {
                var tmp = transA;
                transA = transB;
                transB = tmp;

                var tmpR = shapeRadiusA;
                shapeRadiusA = shapeRadiusB;
                shapeRadiusB = tmpR;
            }

            var manifold = contact.Manifold;
            contact.Evaluate(ref manifold, ref transA, ref transB);

            var pointCount = manifold.PointCount;
            if (pointCount > 0) {
                zeroNormal = manifold.LocalNormal == TSVector2.zero;
                FixedArray2<TSVector2> points;
                WorldManifold.Initialize(ref manifold, ref transA, shapeRadiusA, ref transB, shapeRadiusB,
                    out hitNoraml, out points);

                hitPoint = points[0];

                // Invert the normal if the target shape is a circle.
                if (NotSwapped) {
                    hitNoraml *= -1;
                }
            }
            else {
                hitPoint = TSVector2.zero;
                hitNoraml = TSVector2.up;
            }

            manifold.PointCount = 0;
            contact.Destroy();

            return pointCount > 0;
        }

#if DOD_DEBUG
        private List<Fixture> m_listOverrlapFixture = new List<Fixture>();
        private List<int> m_listOverrlapNodeId = new List<int>();
        private AABB m_casterAAB;

        public void DrawGizmoz()
        {
            var driver = BattleContext.Driver;
            DynamicTreeBroadPhase broad = BroadPhase as DynamicTreeBroadPhase;

            driver.PushGizmozColor(0, 255, 0);
            for (int i = 0; i < m_listOverrlapFixture.Count; i++)
            {
                var fixture = m_listOverrlapFixture[i];
                var shape = fixture.Shape;
                if (shape.ShapeType == ShapeType.Edge)
                {
                    var edageShape = shape as EdgeShape;
                    Transform trans;
                    fixture.Body.GetTransform(out trans);
                    var vert1 = MathUtils.Mul(ref trans, edageShape.Vertex1);
                    var vert2 = MathUtils.Mul(ref trans, edageShape.Vertex2);

                    driver.DrawLine(new TSVector(vert1.x, 2, vert1.y), new TSVector(vert2.x, 2, vert2.y));
                    var currNode = m_listOverrlapNodeId[i];

                    int h = 1;
                    while (currNode >= 0)
                    {
                        driver.PushGizmozColor(0, (byte)(h*40), 0);
                        var aabb = broad.GetBoundAABB(currNode);
                        var size = new TSVector(aabb.Extents.x, FP.Half*h, aabb.Extents.y);
                        driver.DrawGizmozCube(new TSVector(aabb.Center.x, 2, aabb.Center.y), size * 2);
                        currNode = broad.GetDebugParent(currNode);

                        h++;
                    }
                    driver.PopGizmozColor();
                }
            }

            driver.PushGizmozColor(255,255,255);
            if (m_listOverrlapFixture.Count > 0)
            {
                var size = new TSVector(m_casterAAB.Extents.x, 2, m_casterAAB.Extents.y);
                driver.DrawGizmozCube(new TSVector(m_casterAAB.Center.x, 2, m_casterAAB.Center.y), size * 2);
            }
            driver.PopGizmozColor();
            driver.PopGizmozColor();


        }
#endif

        private FixedPoint CircleCastCallbackWrapper(CircleCastInput rayCastInput, int proxyId, int layerMask) {
            FixtureProxy proxy = BroadPhase.GetProxy(proxyId);
            Fixture fixture = proxy.Fixture;
            if (((int) fixture.CollisionCategories & layerMask) == 0) {
                return rayCastInput.MaxFraction;
            }

#if DOD_DEBUG
            m_listOverrlapFixture.Add(fixture);
            m_listOverrlapNodeId.Add(proxyId);
#endif

            var sweepInput = rayCastInput.SweepInput;
            sweepInput.SweepB.LocalCenter = TSVector2.zero;
            sweepInput.SweepB.C0 = sweepInput.SweepB.C = fixture.Body.Position;
            sweepInput.SweepB.A0 = sweepInput.SweepB.A = fixture.Body._sweep.A;
            sweepInput.SweepB.Alpha0 = 0;

            sweepInput.TMax = FixedPoint.One;

            var shape = fixture.Shape;
            var shapeChildCount = shape.ChildCount;
            int contactShapeIndex = -1;
            FixedPoint fraction = rayCastInput.MaxFraction;
            for (int shapeIndex = 0; shapeIndex < shapeChildCount; shapeIndex++) {
                sweepInput.ProxyB.Set(fixture.Shape, shapeIndex);

                TOIOutput sweepOutput;
                TimeOfImpact.CalculateTimeOfImpact(out sweepOutput, sweepInput);

                /*
                 if (!(sweepOutput.state == b2TOIOutput::e_touching ||
                        (sweepOutput.state == b2TOIOutput::e_overlapped && GetPhysics2DSettings ().GetRaycastsStartInColliders ())))
                        continue;
                 */
                if (!(sweepOutput.State == TOIOutputState.Touching ||
                      sweepOutput.State == TOIOutputState.Overlapped)) {
                    continue;
                }

                fraction = sweepOutput.T;
                sweepInput.TMax = fraction;
                contactShapeIndex = shapeIndex;
            }

            if (contactShapeIndex < 0) {
                return rayCastInput.MaxFraction;
            }

            //判断距离
            var output = rayCastInput.Output;
            if (output.fixture != null) {
                if (fixture == output.fixture || fraction > output.fraction) {
                    return rayCastInput.MaxFraction;
                }

                //如果一样的距离，那么优先有normal的
                if (fraction == output.fraction && !output.isZeroNormal) {
                    return rayCastInput.MaxFraction;
                }
            }

            Transform transA;
            sweepInput.SweepA.GetTransform(out transA, fraction);

            Transform transB;
            fixture.Body.GetTransform(out transB);

            //计算距离
            TSVector2 hitPoint;
            TSVector2 hitNormal;

            //var dist = TSVector2.Distance(transA.p, sweepInput.SweepA.C0);
            bool isZeroNormal;
            if (!ConvertHitPointAndNormal(transA, transB, m_castFixture, fixture, contactShapeIndex,
                rayCastInput.Radius,
                fixture.Shape.Radius, out hitPoint, out hitNormal, out isZeroNormal)) {
                return rayCastInput.MaxFraction;
            }

            //如果是背面，也直接忽略
            var hitVec = hitPoint - rayCastInput.Point1;
            if (TSVector2.Dot(rayCastInput.CastDir, hitVec) <= FixedPoint.Zero) {
                //BLogger.Error("leave hit fraction: {0}, zero noarml:{1}, fixture proxy: {2}, hit vec: {3}",
                //    fraction, isZeroNormal, proxyId, hitVec);
                return rayCastInput.MaxFraction;
            }

            //BLogger.Error("hit fraction: {0}, zero noarml:{1}, fixture proxy: {2}", fraction, isZeroNormal, proxyId);
            output.fraction = fraction;
            output.fixture = fixture;
            output.shapeIndex = contactShapeIndex;
            output.isZeroNormal = isZeroNormal;

            _rayCastCallback(fixture, hitPoint, hitNormal, fraction);
            return rayCastInput.MaxFraction;

            //get point and normal

            //return _rayCastCallback(fixture, transA.p, TSVector2.one, fraction);

#if false
            int index = proxy.ChildIndex;
            RayCastOutput output;
            bool hit = fixture.(out output, ref rayCastInput, index);

            if (hit)
            {
                FP fraction = output.Fraction;
                TSVector2 point = (FP.One - fraction) * rayCastInput.Point1 + fraction * rayCastInput.Point2;
                return _rayCastCallback(fixture, point, output.Normal, fraction);
            }

            return rayCastInput.MaxFraction;
#endif
        }

        /// <summary>
        /// Get the number of broad-phase proxies.
        /// </summary>
        /// <value>The proxy count.</value>
        public int ProxyCount {
            get { return BroadPhase.ProxyCount; }
        }

        /// <summary>
        /// Change the global gravity vector.
        /// </summary>
        /// <value>The gravity.</value>
        public TSVector2 Gravity;

        /// <summary>
        /// Get the world body list.
        /// </summary>
        /// <value>Thehead of the world body list.</value>
        public List<Body> BodyList { get; private set; }

#if USE_AWAKE_BODY_SET
        public HashSet<Body> AwakeBodySet { get; private set; }
        List<Body> AwakeBodyList;
#endif
#if USE_ISLAND_SET
        HashSet<Body> IslandSet;
#endif
#if OPTIMIZE_TOI
        HashSet<Body> TOISet;
#endif
        /// <summary>
        /// If false, the whole simulation stops. It still processes added and removed geometries.
        /// </summary>
        public bool Enabled { get; set; }

        /// <summary>
        /// Add a rigid body.
        /// </summary>
        /// <returns></returns>
        internal void AddBody(Body body) {
            Debug.Assert(!_bodyAddList.Contains(body), "You are adding the same body more than once.");

            if (!_bodyAddList.Contains(body))
                _bodyAddList.Add(body);
        }

        /// <summary>
        /// Destroy a rigid body.
        /// Warning: This automatically deletes all associated shapes and joints.
        /// </summary>
        /// <param name="body">The body.</param>
        public void RemoveBody(Body body) {
            Debug.Assert(!_bodyRemoveList.Contains(body),
                "The body is already marked for removal. You are removing the body more than once.");

            if (!_bodyRemoveList.Contains(body))
                _bodyRemoveList.Add(body);

#if USE_AWAKE_BODY_SET
            if (AwakeBodySet.Contains(body))
            {
                AwakeBodySet.Remove(body);
            }
#endif
        }

        /// <summary>
        /// All adds and removes are cached by the World duing a World step.
        /// To process the changes before the world updates again, call this method.
        /// </summary>
        public void ProcessChanges() {
            ProcessAddedBodies();
            ProcessAddedJoints();

            ProcessRemovedBodies();
            ProcessRemovedJoints();

#if DEBUG && USE_AWAKE_BODY_SET
            foreach (var b in AwakeBodySet)
            {
                Debug.Assert(BodyList.Contains(b));
            }
#endif
        }

        /// <summary>
        /// Query the world for all fixtures that potentially overlap the provided AABB.
        ///
        /// Inside the callback:
        /// Return true: Continues the query
        /// Return false: Terminate the query
        /// </summary>
        /// <param name="callback">A user implemented callback class.</param>
        /// <param name="aabb">The aabb query box.</param>
        public void QueryAABB(Func<Fixture, bool> callback, ref AABB aabb) {
            _queryAABBCallback = callback;
            BroadPhase.Query(_queryAABBCallbackWrapper, ref aabb);
            _queryAABBCallback = null;
        }

        /// <summary>
        /// Query the world for all fixtures that potentially overlap the provided AABB.
        /// Use the overload with a callback for filtering and better performance.
        /// </summary>
        /// <param name="aabb">The aabb query box.</param>
        /// <returns>A list of fixtures that were in the affected area.</returns>
        public List<Fixture> QueryAABB(ref AABB aabb) {
            List<Fixture> affected = new List<Fixture>();

            QueryAABB(fixture => {
                affected.Add(fixture);
                return true;
            }, ref aabb);

            return affected;
        }

        /// <summary>
        /// Ray-cast the world for all fixtures in the path of the ray. Your callback
        /// controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        ///
        /// Inside the callback:
        /// return -1: ignore this fixture and continue
        /// return 0: terminate the ray cast
        /// return fraction: clip the ray to this point
        /// return 1: don't clip the ray and continue
        /// </summary>
        /// <param name="callback">A user implemented callback class.</param>
        /// <param name="point1">The ray starting point.</param>
        /// <param name="point2">The ray ending point.</param>
        public void RayCast(Func<Fixture, TSVector2, TSVector2, FixedPoint, FixedPoint> callback,
            TSVector2 point1, TSVector2 point2, int layerMask = (int) Category.All) {
            RayCastInput input = new RayCastInput();
            input.MaxFraction = FixedPoint.One;
            input.Point1 = point1;
            input.Point2 = point2;

            _rayCastCallback = callback;
            BroadPhase.RayCast(_rayCastCallbackWrapper, ref input, layerMask);
            _rayCastCallback = null;
        }

        private Fixture m_castFixture = new Fixture();
        private TOIInput m_sweepInput = new TOIInput();
        private Shape m_circleShape = new CircleShape();
        private CircleCastOutput m_output = new CircleCastOutput();

        public void CircleCast(Func<Fixture, TSVector2, TSVector2, FixedPoint, FixedPoint> callback,
            TSVector2 point1, TSVector2 dir, FixedPoint dist, FixedPoint radius, int layerMask = (int) Category.All) {
            var input = new CircleCastInput();
            input.MaxFraction = FixedPoint.One;
            input.Point1 = point1;
            input.Point2 = point1 + dir * dist;
            input.Radius = radius;
            input.CastDir = dir;
            input.SweepInput = m_sweepInput;
            input.Output = m_output;
            m_output.fixture = null;
            m_output.fraction = FixedPoint.One;
            m_output.shapeIndex = -1;

            InitSweep(m_sweepInput, point1, input.Point2, radius);

            //sweep.ProxyA
#if DOD_DEBUG
            m_listOverrlapFixture.Clear();
            m_listOverrlapNodeId.Clear();

            AABB segmentAABB = new AABB();
            {
                TSVector2 t = input.Point2;
                TSVector2.Min(ref point1, ref t, out segmentAABB.LowerBound);
                TSVector2.Max(ref point1, ref t, out segmentAABB.UpperBound);

                segmentAABB.LowerBound -= new TSVector2(input.Radius);
                segmentAABB.UpperBound += new TSVector2(input.Radius);
            }
            m_casterAAB = segmentAABB;
#endif

            //BLogger.Error("start raycast----------------");
            _rayCastCallback = callback;
            BroadPhase.CircleCast(_circleCastCallbackWrapper, ref input, layerMask);
            _rayCastCallback = null;

            //if (m_output.fixture != null)
            //{
            //    BLogger.Error("CircleCast finish");
            //}
        }

        private void InitSweep(TOIInput sweepInput, TSVector2 p1, TSVector2 p2, FixedPoint radius) {
            m_circleShape.Radius = radius;
            m_castFixture.Initialize(m_circleShape, false);

            sweepInput.ProxyA.Set(m_circleShape, 0);
            sweepInput.SweepA.LocalCenter = TSVector2.zero;
            sweepInput.SweepA.C0 = p1;
            sweepInput.SweepA.C = p2;
            sweepInput.SweepA.A0 = sweepInput.SweepA.A = 0;
            sweepInput.SweepA.Alpha0 = 0;
        }

        public Fixture TestPoint(TSVector2 point) {
            AABB aabb;
            TSVector2 d = new TSVector2(Settings.Epsilon, Settings.Epsilon);
            aabb.LowerBound = point - d;
            aabb.UpperBound = point + d;

            _myFixture = null;
            _point1 = point;

            // Query the world for overlapping shapes.
            QueryAABB(TestPointCallback, ref aabb);

            return _myFixture;
        }

        private bool TestPointCallback(Fixture fixture) {
            bool inside = fixture.TestPoint(ref _point1);
            if (inside) {
                _myFixture = fixture;
                return false;
            }

            // Continue the query.
            return true;
        }

        /// <summary>
        /// Returns a list of fixtures that are at the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns></returns>
        public List<Fixture> TestPointAll(TSVector2 point) {
            AABB aabb;
            TSVector2 d = new TSVector2(Settings.Epsilon, Settings.Epsilon);
            aabb.LowerBound = point - d;
            aabb.UpperBound = point + d;

            _point2 = point;
            _testPointAllFixtures = new List<Fixture>();

            // Query the world for overlapping shapes.
            QueryAABB(TestPointAllCallback, ref aabb);

            return _testPointAllFixtures;
        }

        private bool TestPointAllCallback(Fixture fixture) {
            bool inside = fixture.TestPoint(ref _point2);
            if (inside)
                _testPointAllFixtures.Add(fixture);

            // Continue the query.
            return true;
        }

        public void Clear() {
            ProcessChanges();

            for (int i = BodyList.Count - 1; i >= 0; i--) {
                RemoveBody(BodyList[i]);
            }

            ProcessChanges();
        }

        public List<IBody> Bodies() {
            List<IBody> bodies = new List<IBody>();
            for (int index = 0; index < BodyList.Count; index++) {
                bodies.Add(BodyList[index]);
            }

            return bodies;
        }
    }
}