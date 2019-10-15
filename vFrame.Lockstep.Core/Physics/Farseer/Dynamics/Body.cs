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
//#define USE_AWAKE_BODY_SET

using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{

    public enum BodySpecialSensor {
        None,
        ActiveOnce,
        ActiveAll
    }

    /// <summary>
    /// The body type.
    /// </summary>
    public enum BodyType
    {
        /// <summary>
        /// Zero velocity, may be manually moved. Note: even static bodies have mass.
        /// </summary>
        Static,
        /// <summary>
        /// Positive mass, non-zero velocity determined by forces, moved by solver
        /// </summary>
        Dynamic,
    }

    public class Body : IDisposable, IBody2D
    {
        [ThreadStatic]
        internal static int _bodyIdCounter;

        internal BodySpecialSensor _specialSensor = BodySpecialSensor.None;
        public int SpecialSensorMask = -1;

        public List<Body> _specialSensorResults;

        public BodySpecialSensor SpecialSensor {
            get {
                return _specialSensor;
            }
            set {
                _specialSensor = value;

                if (value != BodySpecialSensor.None) {
                    _specialSensorResults = new List<Body>();
                }
            }
        }

        internal BodyType _bodyType;
        
        internal bool _sleepingAllowed = true;
        internal bool _awake = true;

        internal bool _enabled = true;
        internal FP _sleepTime;
        internal Sweep _sweep; // the swept motion for CCD

        internal World _world;
        internal Transform _xf; // the body origin transform

        internal bool disabled;
        
        public List<IBodyConstraint> bodyConstraints;

        public PhysicsLogicFilter PhysicsLogicFilter;

        // TS - public Body(World world, Vector2? position = null, FP rotation = 0, object userdata = null)
        public Body(World world, TSVector2? position, TSVector2 forward, object userdata = null)
        {
            FixtureList = new List<Fixture>();
            bodyConstraints = new List<IBodyConstraint>();
            BodyId = _bodyIdCounter++;
            _world = world;

            UserData = userdata;
            GravityScale = FP.One;
            BodyType = BodyType.Static;
            Enabled = true; //FPE note: Also creates proxies in the broadphase

            _xf.q.Set(forward);

            if (position.HasValue)
            {
                _xf.p = position.Value;
                _sweep.C0 = _xf.p;
                _sweep.C = _xf.p;
                _sweep.A0 = _sweep.A = GetAngle(forward);
            }

            world.AddBody(this); //FPE note: bodies can't live without a World
        }

        private FP GetAngle(TSVector2 dir)
        {
            var ret = TSMath.Atan2(dir.y, dir.x);
            var pi2 = TSMath.Pi * 2;
            if (ret > pi2)
            {
                ret -= pi2;
            }
            if (ret < 0)
            {
                ret += pi2;
            }

            return ret;
        }

        /// <summary>
        /// A unique id for this body.
        /// </summary>
        public int BodyId { get; private set; }

        public int IslandIndex { get; set; }

        /// <summary>
        /// Scale the gravity applied to this body.
        /// Defaults to 1. A value of 2 means FP the gravity is applied to this body.
        /// </summary>
        public FP GravityScale { get; set; }

        /// <summary>
        /// Set the user data. Use this to store your application specific data.
        /// </summary>
        /// <value>The user data.</value>
        public object UserData { get; set; }
        
        /// <summary>
        /// Gets or sets the body type.
        /// Warning: Calling this mid-update might cause a crash.
        /// </summary>
        /// <value>The type of body.</value>
        public BodyType BodyType
        {
            get { return _bodyType; }
            set
            {
                if (_bodyType == value)
                    return;

                BodyType lastType = _bodyType;

                _bodyType = value;
                SynchronizeFixtures();
                Awake = true;
                
                // Touch the proxies so that new contacts will be created (when appropriate)
                IBroadPhase broadPhase = _world.BroadPhase;
                foreach (Fixture fixture in FixtureList)
                {
                    int proxyCount = fixture.ProxyCount;
                    for (int j = 0; j < proxyCount; j++)
                    {
                        broadPhase.TouchProxy(fixture.Proxies[j].ProxyId);
                    }
                }
            }
        }
        
        /// <summary>
        /// You can disable sleeping on this body. If you disable sleeping, the
        /// body will be woken.
        /// </summary>
        /// <value><c>true</c> if sleeping is allowed; otherwise, <c>false</c>.</value>
        public bool SleepingAllowed
        {
            set
            {
                if (!value)
                    Awake = true;

                _sleepingAllowed = value;
            }
            get { return _sleepingAllowed; }
        }

        /// <summary>
        /// Set the sleep state of the body. A sleeping body has very
        /// low CPU cost.
        /// </summary>
        /// <value><c>true</c> if awake; otherwise, <c>false</c>.</value>
        public bool Awake
        {
            set
            {
                if (value)
                {
                    if (!_awake)
                    {
                        _sleepTime = FP.Zero;

#if USE_AWAKE_BODY_SET
						if (InWorld && !World.AwakeBodySet.Contains(this))
						{
							World.AwakeBodySet.Add(this);
						}
#endif
                    }
                }
                else
                {
#if USE_AWAKE_BODY_SET
					// Check even for BodyType.Static because if this body had just been changed to Static it will have
					// set Awake = false in the process.
					if (InWorld && World.AwakeBodySet.Contains(this))
					{
						World.AwakeBodySet.Remove(this);
					}
#endif
                    ResetDynamics();
                    _sleepTime = FP.Zero;
                }

                _awake = value;
            }
            get { return _awake; }
        }

        /// <summary>
        /// Set the active state of the body. An inactive body is not
        /// simulated and cannot be collided with or woken up.
        /// If you pass a flag of true, all fixtures will be added to the
        /// broad-phase.
        /// If you pass a flag of false, all fixtures will be removed from
        /// the broad-phase and all contacts will be destroyed.
        /// Fixtures and joints are otherwise unaffected. You may continue
        /// to create/destroy fixtures and joints on inactive bodies.
        /// Fixtures on an inactive body are implicitly inactive and will
        /// not participate in collisions, ray-casts, or queries.
        /// Joints connected to an inactive body are implicitly inactive.
        /// An inactive body is still owned by a b2World object and remains
        /// in the body list.
        /// </summary>
        /// <value><c>true</c> if active; otherwise, <c>false</c>.</value>
        public bool Enabled
        {
            set
            {
                if (value == _enabled)
                    return;

                if (value)
                {
                    // Create all proxies.
                    IBroadPhase broadPhase = _world.BroadPhase;
                    for (int i = 0; i < FixtureList.Count; i++)
                    {
                        FixtureList[i].CreateProxies(broadPhase, ref _xf);
                    }

                    // Contacts are created the next time step.
                }
                else
                {
                    // Destroy all proxies.
                    IBroadPhase broadPhase = _world.BroadPhase;

                    for (int i = 0; i < FixtureList.Count; i++)
                    {
                        FixtureList[i].DestroyProxies(broadPhase);
                    }
                }

                _enabled = value;
            }
            get { return _enabled; }
        }
        
        /// <summary>
        /// Gets all the fixtures attached to this body.
        /// </summary>
        /// <value>The fixture list.</value>
        public List<Fixture> FixtureList { get; internal set; }
        
        /// <summary>
        /// Get the world body origin position.
        /// </summary>
        /// <returns>Return the world position of the body's origin.</returns>
        public TSVector2 Position
        {
            get { return _xf.p; }
            set
            {
                Debug.Assert(!FP.IsNaN(value.x) && !FP.IsNaN(value.y));

                var forward = Forward;
                SetTransform(ref value, ref forward);
            }
        }

        ///// <summary>
        ///// Get the angle in radians.
        ///// </summary>
        ///// <returns>Return the current world rotation angle in radians.</returns>
        //public FP Rotation
        //{
        //    get { return _sweep.A; }
        //    set
        //    {
        //        Debug.Assert(!FP.IsNaN(value));

        //        SetTransform(ref _xf.p, value);
        //    }
        //}

        public TSVector2 Forward
        {
            get
            {
                return new TSVector2(_xf.q.c, _xf.q.s);
            }
            set
            {
                SetTransform(ref _xf.p, ref value);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether this body is static.
        /// </summary>
        /// <value><c>true</c> if this instance is static; otherwise, <c>false</c>.</value>
        public bool IsStatic
        {
            get { return _bodyType == BodyType.Static; }
            set { BodyType = value ? BodyType.Static : BodyType.Dynamic; }
        }

        public bool TSIsStatic {
            get {
                return IsStatic;
            }
            set {
                this.IsStatic = value;
            }
        }
        
        public FP Friction
        {
            get
            {
                FP res = 0;

                for (int i = 0; i < FixtureList.Count; i++)
                {
                    Fixture f = FixtureList[i];
                    res += f.Friction;
                }

                return FixtureList.Count > 0 ? res / FixtureList.Count : 0;
            }
            set
            {
                for (int i = 0; i < FixtureList.Count; i++)
                {
                    Fixture f = FixtureList[i];
                    f.Friction = value;
                }
            }
        }

        public Category CollisionCategories
        {
            set
            {
                for (int i = 0; i < FixtureList.Count; i++)
                {
                    Fixture f = FixtureList[i];
                    f.CollisionCategories = value;
                }
            }
        }
        
        public short CollisionGroup
        {
            set
            {
                for (int i = 0; i < FixtureList.Count; i++)
                {
                    Fixture f = FixtureList[i];
                    f.CollisionGroup = value;
                }
            }
        }

        public bool IsSensor {
            get {
                return FixtureList[0].IsSensor;
            }

            set
            {
                for (int i = 0; i < FixtureList.Count; i++)
                {
                    Fixture f = FixtureList[i];
                    f.IsSensor = value;
                }
            }
        }
        
        /// <summary>
        /// Resets the dynamics of this body.
        /// Sets torque, force and linear/angular velocity to 0
        /// </summary>
        public void ResetDynamics()
        {
        }

        /// <summary>
        /// Creates a fixture and attach it to this body.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// Contacts are not created until the next time step.
        /// Warning: This function is locked during callbacks.
        /// </summary>
        /// <param name="shape">The shape.</param>
        /// <param name="userData">Application specific data</param>
        /// <returns></returns>
        public Fixture CreateFixture(Shape shape, object userData = null)
        {
            return new Fixture(this, shape, userData);
        }

        /// <summary>
        /// Destroy a fixture. This removes the fixture from the broad-phase and
        /// destroys all contacts associated with this fixture. This will
        /// automatically adjust the mass of the body if the body is dynamic and the
        /// fixture has positive density.
        /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
        /// Warning: This function is locked during callbacks.
        /// </summary>
        /// <param name="fixture">The fixture to be removed.</param>
        public void DestroyFixture(Fixture fixture)
        {
            Debug.Assert(fixture.Body == this);

            // Remove the fixture from this body's singly linked list.
            Debug.Assert(FixtureList.Count > 0);

            // You tried to remove a fixture that not present in the fixturelist.
            Debug.Assert(FixtureList.Contains(fixture));

            if (_enabled)
            {
                IBroadPhase broadPhase = _world.BroadPhase;
                fixture.DestroyProxies(broadPhase);
            }

            FixtureList.Remove(fixture);
            fixture.Destroy();
            fixture.Body = null;
        }

        /// <summary>
        /// Set the position of the body's origin and rotation.
        /// This breaks any contacts and wakes the other bodies.
        /// Manipulating a body's transform may cause non-physical behavior.
        /// </summary>
        /// <param name="position">The world position of the body's local origin.</param>
        /// <param name="rotation">The world rotation in radians.</param>
        public void SetTransform(ref TSVector2 position, FP rotation)
        {
            SetTransformIgnoreContacts(ref position, rotation);
        }

        public void SetTransform(ref TSVector2 position, ref TSVector2 forward)
        {
            SetTransformIgnoreContacts(ref position, ref forward);
        }
        
        /// <summary>
        /// For teleporting a body without considering new contacts immediately.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The angle.</param>
        public void SetTransformIgnoreContacts(ref TSVector2 position, FP angle)
        {
            _xf.q.Set(angle);
            _xf.p = position;

            ApplyTransformChanged();
        }

        /// <summary>
        /// For teleporting a body without considering new contacts immediately.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The angle.</param>
        public void SetTransformIgnoreContacts(ref TSVector2 position, ref TSVector2 forward)
        {
            _xf.q.Set(forward);
            _xf.p = position;
            ApplyTransformChanged();
        }

        private void ApplyTransformChanged()
        {
            IBroadPhase broadPhase = _world.BroadPhase;
            for (int i = 0; i < FixtureList.Count; i++)
            {
                FixtureList[i].Synchronize(broadPhase, ref _xf);
            }
        }

        /// <summary>
        /// Get the body transform for the body's origin.
        /// </summary>
        /// <param name="transform">The transform of the body's origin.</param>
        public void GetTransform(out Transform transform)
        {
            transform = _xf;
        }
        
        /// <summary>
        /// Get the world coordinates of a point given the local coordinates.
        /// </summary>
        /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
        /// <returns>The same point expressed in world coordinates.</returns>
        public TSVector2 GetWorldPoint(ref TSVector2 localPoint)
        {
            return MathUtils.Mul(ref _xf, ref localPoint);
        }

        /// <summary>
        /// Get the world coordinates of a point given the local coordinates.
        /// </summary>
        /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
        /// <returns>The same point expressed in world coordinates.</returns>
        public TSVector2 GetWorldPoint(TSVector2 localPoint)
        {
            return GetWorldPoint(ref localPoint);
        }

        /// <summary>
        /// Get the world coordinates of a vector given the local coordinates.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="localVector">A vector fixed in the body.</param>
        /// <returns>The same vector expressed in world coordinates.</returns>
        public TSVector2 GetWorldVector(ref TSVector2 localVector)
        {
            return MathUtils.Mul(_xf.q, localVector);
        }

        /// <summary>
        /// Get the world coordinates of a vector given the local coordinates.
        /// </summary>
        /// <param name="localVector">A vector fixed in the body.</param>
        /// <returns>The same vector expressed in world coordinates.</returns>
        public TSVector2 GetWorldVector(TSVector2 localVector)
        {
            return GetWorldVector(ref localVector);
        }

        /// <summary>
        /// Gets a local point relative to the body's origin given a world point.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The corresponding local point relative to the body's origin.</returns>
        public TSVector2 GetLocalPoint(ref TSVector2 worldPoint)
        {
            return MathUtils.MulT(ref _xf, worldPoint);
        }

        /// <summary>
        /// Gets a local point relative to the body's origin given a world point.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The corresponding local point relative to the body's origin.</returns>
        public TSVector2 GetLocalPoint(TSVector2 worldPoint)
        {
            return GetLocalPoint(ref worldPoint);
        }

        /// <summary>
        /// Gets a local vector given a world vector.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldVector">A vector in world coordinates.</param>
        /// <returns>The corresponding local vector.</returns>
        public TSVector2 GetLocalVector(ref TSVector2 worldVector)
        {
            return MathUtils.MulT(_xf.q, worldVector);
        }

        /// <summary>
        /// Gets a local vector given a world vector.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldVector">A vector in world coordinates.</param>
        /// <returns>The corresponding local vector.</returns>
        public TSVector2 GetLocalVector(TSVector2 worldVector)
        {
            return GetLocalVector(ref worldVector);
        }
        
        internal void SynchronizeFixtures()
        {
            IBroadPhase broadPhase = _world.BroadPhase;
            for (int i = 0; i < FixtureList.Count; i++)
            {
                FixtureList[i].Synchronize(broadPhase, ref _xf);
            }
        }

        #region IDisposable Members

        public bool IsDisposed { get; set; }
        
        public bool TSDisabled {
            get {
                return disabled;
            }

            set {
                disabled = value;
            }
        }

        public TSVector2 TSPosition {
            get {
                return Position;
            }

            set {
                Position = value;
            }
        }


        public FP TSFriction {
            get {
                return FixtureList[0].Friction;
            }

            set {
                List<Fixture> fixtures = FixtureList;

                for (int index = 0, length = fixtures.Count; index < length; index++) {
                    fixtures[index].Friction = value;
                }
            }
        }

        public FP TSRestitution {
            get {
                return FixtureList[0].Restitution;
            }

            set {
                List<Fixture> fixtures = FixtureList;

                for (int index = 0, length = fixtures.Count; index < length; index++) {
                    fixtures[index].Restitution = value;
                }
            }
        }

        public void Dispose()
        {
            if (!IsDisposed)
            {
                _world.RemoveBody(this);
                IsDisposed = true;
                GC.SuppressFinalize(this);
            }
        }

        #endregion

        /// <summary>
        /// Makes a clone of the body. Fixtures and therefore shapes are not included.
        /// Use DeepClone() to clone the body, as well as fixtures and shapes.
        /// </summary>
        /// <param name="world"></param>
        /// <returns></returns>
        public Body Clone(World world = null)
        {
            Body body = new Body(world ?? _world, Position, Forward, UserData);
            body._bodyType = _bodyType;
            body.GravityScale = GravityScale;
            body.UserData = UserData;
            body._enabled = _enabled;
            body._sleepingAllowed = _sleepingAllowed;
            body._awake = _awake;

            return body;
        }

        /// <summary>
        /// Clones the body and all attached fixtures and shapes. Simply said, it makes a complete copy of the body.
        /// </summary>
        /// <param name="world"></param>
        /// <returns></returns>
        public Body DeepClone(World world = null)
        {
            Body body = Clone(world ?? _world);

            int count = FixtureList.Count; //Make a copy of the count. Otherwise it causes an infinite loop.
            for (int i = 0; i < count; i++)
            {
                FixtureList[i].CloneOnto(body);
            }

            return body;
        }

        public string Checkum() {
            return Position + "|" + Forward;
        }
    }

}