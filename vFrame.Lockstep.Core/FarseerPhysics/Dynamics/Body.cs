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
//#define USE_AWAKE_BODY_SET

using System;
using System.Collections.Generic;
using System.Diagnostics;
using vFrame.Lockstep.Core.FarseerPhysics.Collision;
using vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes;
using vFrame.Lockstep.Core.FarseerPhysics.Common;
using vFrame.Lockstep.Core.FarseerPhysics.Common.PhysicsLogic;
using vFrame.Lockstep.Core.FarseerPhysics.Controllers;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Contacts;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints;

namespace vFrame.Lockstep.Core.FarseerPhysics.Dynamics
{
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
        /// Zero mass, non-zero velocity set by user, moved by solver
        /// </summary>
        Kinematic,

        /// <summary>
        /// Positive mass, non-zero velocity determined by forces, moved by solver
        /// </summary>
        Dynamic,
    }

    [Flags]
    public enum BodyFlags
    {
        None = 0,
        Island = (1 << 0),
        Awake = (1 << 1),
        AutoSleep = (1 << 2),
        Bullet = (1 << 3),
        FixedRotation = (1 << 4),
        Enabled = (1 << 5),
        IgnoreGravity = (1 << 6),
        IgnoreCCD = (1 << 7),
    }

    public class Body : IDisposable
    {
        private static int _bodyIdCounter;
        internal FixedPoint AngularVelocityInternal;
        public int BodyId;
        public ControllerFilter ControllerFilter;
        internal BodyFlags Flags;
        internal TSVector2 Force;
        internal FixedPoint InvI;
        internal FixedPoint InvMass;
        internal TSVector2 LinearVelocityInternal;
        public PhysicsLogicFilter PhysicsLogicFilter;
        internal FixedPoint SleepTime;
        internal Sweep Sweep; // the swept motion for CCD
        internal FixedPoint Torque;
        internal World World;
        internal Transform Xf; // the body origin transform
        private FixedPoint _angularDamping;
        private BodyType _bodyType;
        private FixedPoint _inertia;
        private FixedPoint _linearDamping;
        private FixedPoint _mass;
        public int IslandIndex;

        /// Scale the gravity applied to this body.
        public FixedPoint GravityScale { get; set; }

        internal Body() {
            FixtureList = new List<Fixture>(32);
        }

        public Body(World world)
            : this(world, null) {
        }

        public Body(World world, object userData) {
            FixtureList = new List<Fixture>(32);
            BodyId = _bodyIdCounter++;

            World = world;
            UserData = userData;

            GravityScale = 1.0f;
            FixedRotation = false;
            IsBullet = false;
            SleepingAllowed = true;
#if !USE_AWAKE_BODY_SET
            Awake = true;
#endif
            BodyType = BodyType.Static;
            Enabled = true;

            Xf.q.Set(0);

            world.AddBody(this);
        }

        /// <summary>
        /// Gets the total number revolutions the body has made.
        /// </summary>
        /// <value>The revolutions.</value>
        public FixedPoint Revolutions {
            get { return Rotation / (FixedPoint) TSMath.Pi; }
        }

        /// <summary>
        /// Gets or sets the body type.
        /// </summary>
        /// <value>The type of body.</value>
        public BodyType BodyType {
            get { return _bodyType; }
            set {
                if (_bodyType == value) {
                    return;
                }

                _bodyType = value;

                ResetMassData();

                if (_bodyType == BodyType.Static) {
                    LinearVelocityInternal = TSVector2.zero;
                    AngularVelocityInternal = 0.0f;
                    Sweep.A0 = Sweep.A;
                    Sweep.C0 = Sweep.C;
                    SynchronizeFixtures();
                }

                Awake = true;

                Force = TSVector2.zero;
                Torque = 0.0f;

                // Delete the attached contacts.
                ContactEdge ce = ContactList;
                while (ce != null) {
                    ContactEdge ce0 = ce;
                    ce = ce.Next;
                    World.ContactManager.Destroy(ce0.Contact);
                }

                ContactList = null;

                // Touch the proxies so that new contacts will be created (when appropriate)
                IBroadPhase broadPhase = World.ContactManager.BroadPhase;
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture fixture = FixtureList[i];
                    int proxyCount = fixture.ProxyCount;
                    for (int j = 0; j < proxyCount; j++) {
                        broadPhase.TouchProxy(fixture.Proxies[j].ProxyId);
                    }
                }

                // Since the body type changed, we need to flag contacts for filtering.
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.Refilter();
                }
            }
        }

        /// <summary>
        /// Get or sets the linear velocity of the center of mass.
        /// </summary>
        /// <value>The linear velocity.</value>
        public TSVector2 LinearVelocity {
            set {
                Debug.Assert(!FixedPoint.IsNaN(value.x) && !FixedPoint.IsNaN(value.y));

                if (_bodyType == BodyType.Static)
                    return;

                if (TSVector2.Dot(value, value) > 0.0f)
                    Awake = true;

                LinearVelocityInternal = value;
            }
            get { return LinearVelocityInternal; }
        }

        /// <summary>
        /// Gets or sets the angular velocity. Radians/second.
        /// </summary>
        /// <value>The angular velocity.</value>
        public FixedPoint AngularVelocity {
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                if (_bodyType == BodyType.Static)
                    return;

                if (value * value > 0.0f)
                    Awake = true;

                AngularVelocityInternal = value;
            }
            get { return AngularVelocityInternal; }
        }

        /// <summary>
        /// Gets or sets the linear damping.
        /// </summary>
        /// <value>The linear damping.</value>
        public FixedPoint LinearDamping {
            get { return _linearDamping; }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                _linearDamping = value;
            }
        }

        /// <summary>
        /// Gets or sets the angular damping.
        /// </summary>
        /// <value>The angular damping.</value>
        public FixedPoint AngularDamping {
            get { return _angularDamping; }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                _angularDamping = value;
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether this body should be included in the CCD solver.
        /// </summary>
        /// <value><c>true</c> if this instance is included in CCD; otherwise, <c>false</c>.</value>
        public bool IsBullet {
            set {
                if (value) {
                    Flags |= BodyFlags.Bullet;
                }
                else {
                    Flags &= ~BodyFlags.Bullet;
                }
            }
            get { return (Flags & BodyFlags.Bullet) == BodyFlags.Bullet; }
        }

        /// <summary>
        /// You can disable sleeping on this body. If you disable sleeping, the
        /// body will be woken.
        /// </summary>
        /// <value><c>true</c> if sleeping is allowed; otherwise, <c>false</c>.</value>
        public bool SleepingAllowed {
            set {
                if (value) {
                    Flags |= BodyFlags.AutoSleep;
                }
                else {
                    Flags &= ~BodyFlags.AutoSleep;
                    Awake = true;
                }
            }
            get { return (Flags & BodyFlags.AutoSleep) == BodyFlags.AutoSleep; }
        }

        /// <summary>
        /// Set the sleep state of the body. A sleeping body has very
        /// low CPU cost.
        /// </summary>
        /// <value><c>true</c> if awake; otherwise, <c>false</c>.</value>
        public bool Awake {
            set {
                if (value) {
                    if ((Flags & BodyFlags.Awake) == 0) {
                        Flags |= BodyFlags.Awake;
                        SleepTime = 0.0f;
                        World.ContactManager.UpdateContacts(ContactList, true);
#if USE_AWAKE_BODY_SET
						if (InWorld && !World.AwakeBodySet.Contains(this))
						{
							World.AwakeBodySet.Add(this);
						}
#endif
                    }
                }
                else {
#if USE_AWAKE_BODY_SET
					// Check even for BodyType.Static because if this body had just been changed to Static it will have
					// set Awake = false in the process.
					if (InWorld && World.AwakeBodySet.Contains(this))
					{
						World.AwakeBodySet.Remove(this);
					}
#endif
                    Flags &= ~BodyFlags.Awake;
                    SleepTime = 0.0f;
                    LinearVelocityInternal = TSVector2.zero;
                    AngularVelocityInternal = 0.0f;
                    Force = TSVector2.zero;
                    Torque = 0.0f;
                    World.ContactManager.UpdateContacts(ContactList, false);
                }
            }
            get { return (BodyType != BodyType.Static) && (Flags & BodyFlags.Awake) == BodyFlags.Awake; }
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
        public bool Enabled {
            set {
                if (value == Enabled) {
                    return;
                }

                if (value) {
                    Flags |= BodyFlags.Enabled;

                    // Create all proxies.
                    IBroadPhase broadPhase = World.ContactManager.BroadPhase;
                    for (int i = 0; i < FixtureList.Count; i++) {
                        FixtureList[i].CreateProxies(broadPhase, ref Xf);
                    }

                    // Contacts are created the next time step.
                }
                else {
                    Flags &= ~BodyFlags.Enabled;

                    // Destroy all proxies.
                    IBroadPhase broadPhase = World.ContactManager.BroadPhase;

                    for (int i = 0; i < FixtureList.Count; i++) {
                        FixtureList[i].DestroyProxies(broadPhase);
                    }

                    // Destroy the attached contacts.
                    ContactEdge ce = ContactList;
                    while (ce != null) {
                        ContactEdge ce0 = ce;
                        ce = ce.Next;
                        World.ContactManager.Destroy(ce0.Contact);
                    }

                    ContactList = null;
                }
            }
            get { return (Flags & BodyFlags.Enabled) == BodyFlags.Enabled; }
        }

        /// <summary>
        /// Set this body to have fixed rotation. This causes the mass
        /// to be reset.
        /// </summary>
        /// <value><c>true</c> if it has fixed rotation; otherwise, <c>false</c>.</value>
        public bool FixedRotation {
            set {
                if (value) {
                    Flags |= BodyFlags.FixedRotation;
                }
                else {
                    Flags &= ~BodyFlags.FixedRotation;
                }

                ResetMassData();
            }
            get { return (Flags & BodyFlags.FixedRotation) == BodyFlags.FixedRotation; }
        }

        public bool InWorld { get; internal set; }

        /// <summary>
        /// Gets all the fixtures attached to this body.
        /// </summary>
        /// <value>The fixture list.</value>
        public List<Fixture> FixtureList { get; internal set; }

        /// <summary>
        /// Get the list of all joints attached to this body.
        /// </summary>
        /// <value>The joint list.</value>
        public JointEdge JointList { get; internal set; }

        /// <summary>
        /// Get the list of all contacts attached to this body.
        /// Warning: this list changes during the time step and you may
        /// miss some collisions if you don't use ContactListener.
        /// </summary>
        /// <value>The contact list.</value>
        public ContactEdge ContactList { get; internal set; }

        /// <summary>
        /// Set the user data. Use this to store your application specific data.
        /// </summary>
        /// <value>The user data.</value>
        public object UserData { get; set; }

        // GABS
        /// <summary>
        /// Gets or sets the user tag.
        /// </summary>
        /// <value>
        /// The user tag.
        /// </value>
        public string UserTag { get; set; }

        /// <summary>
        /// Get the world body origin position.
        /// </summary>
        /// <returns>Return the world position of the body's origin.</returns>
        public TSVector2 Position {
            get { return Xf.p; }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value.x) && !FixedPoint.IsNaN(value.y));

                SetTransform(ref value, Rotation);
            }
        }

        /// <summary>
        /// Get the angle in radians.
        /// </summary>
        /// <returns>Return the current world rotation angle in radians.</returns>
        public FixedPoint Rotation {
            get { return Sweep.A; }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                SetTransform(ref Xf.p, value);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether this body is static.
        /// </summary>
        /// <value><c>true</c> if this instance is static; otherwise, <c>false</c>.</value>
        public bool IsStatic {
            get { return _bodyType == BodyType.Static; }
            set {
                if (value)
                    BodyType = BodyType.Static;
                else
                    BodyType = BodyType.Dynamic;
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether this body ignores gravity.
        /// </summary>
        /// <value><c>true</c> if  it ignores gravity; otherwise, <c>false</c>.</value>
        public bool IgnoreGravity {
            get { return (Flags & BodyFlags.IgnoreGravity) == BodyFlags.IgnoreGravity; }
            set {
                if (value)
                    Flags |= BodyFlags.IgnoreGravity;
                else
                    Flags &= ~BodyFlags.IgnoreGravity;
            }
        }

        /// <summary>
        /// Get the world position of the center of mass.
        /// </summary>
        /// <value>The world position.</value>
        public TSVector2 WorldCenter {
            get { return Sweep.C; }
        }

        /// <summary>
        /// Get the local position of the center of mass.
        /// </summary>
        /// <value>The local position.</value>
        public TSVector2 LocalCenter {
            get { return Sweep.LocalCenter; }
            set {
                if (_bodyType != BodyType.Dynamic)
                    return;

                // Move center of mass.
                TSVector2 oldCenter = Sweep.C;
                Sweep.LocalCenter = value;
                Sweep.C0 = Sweep.C = MathUtils.Mul(ref Xf, ref Sweep.LocalCenter);

                // Update center of mass velocity.
                TSVector2 a = Sweep.C - oldCenter;
                LinearVelocityInternal += new TSVector2(-AngularVelocityInternal * a.y, AngularVelocityInternal * a.x);
            }
        }

        /// <summary>
        /// Gets or sets the mass. Usually in kilograms (kg).
        /// </summary>
        /// <value>The mass.</value>
        public FixedPoint Mass {
            get { return _mass; }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                if (_bodyType != BodyType.Dynamic)
                    return;

                _mass = value;

                if (_mass <= 0.0f)
                    _mass = 1.0f;

                InvMass = 1.0f / _mass;
            }
        }

        /// <summary>
        /// Get or set the rotational inertia of the body about the local origin. usually in kg-m^2.
        /// </summary>
        /// <value>The inertia.</value>
        public FixedPoint Inertia {
            get { return _inertia + Mass * TSVector2.Dot(Sweep.LocalCenter, Sweep.LocalCenter); }
            set {
                Debug.Assert(!FixedPoint.IsNaN(value));

                if (_bodyType != BodyType.Dynamic)
                    return;

                if (value > 0.0f && (Flags & BodyFlags.FixedRotation) == 0) {
                    _inertia = value - Mass * TSVector2.Dot(LocalCenter, LocalCenter);
                    Debug.Assert(_inertia > 0.0f);
                    InvI = 1.0f / _inertia;
                }
            }
        }

        public FixedPoint Restitution {
            //TODO: Fix bug when fixturecount = 0;
            get {
                FixedPoint res = 0;

                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    res += f.Restitution;
                }

                return res / FixtureList.Count;
            }
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.Restitution = value;
                }
            }
        }

        public FixedPoint Friction {
            //TODO: Fix bug when fixturecount = 0;
            get {
                FixedPoint res = 0;

                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    res += f.Friction;
                }

                return res / FixtureList.Count;
            }
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.Friction = value;
                }
            }
        }

        public Category CollisionCategories {
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.CollisionCategories = value;
                }
            }
        }

        public Category CollidesWith {
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.CollidesWith = value;
                }
            }
        }

        /// <summary>
        /// Body objects can define which categories of bodies they wish to ignore CCD with.
        /// This allows certain bodies to be configured to ignore CCD with objects that
        /// aren't a penetration problem due to the way content has been prepared.
        /// This is compared against the other Body's fixture CollisionCategories within World.SolveTOI().
        /// </summary>
        public Category IgnoreCCDWith {
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.IgnoreCCDWith = value;
                }
            }
        }

        public short CollisionGroup {
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.CollisionGroup = value;
                }
            }
        }

        public bool IsSensor {
            set {
                for (int i = 0; i < FixtureList.Count; i++) {
                    Fixture f = FixtureList[i];
                    f.IsSensor = value;
                }
            }
        }

        public bool IgnoreCCD {
            get { return (Flags & BodyFlags.IgnoreCCD) == BodyFlags.IgnoreCCD; }
            set {
                if (value)
                    Flags |= BodyFlags.IgnoreCCD;
                else
                    Flags &= ~BodyFlags.IgnoreCCD;
            }
        }

        #region IDisposable Members

        public bool IsDisposed { get; set; }

        public void Dispose() {
            if (!IsDisposed) {
                World.RemoveBody(this);
                IsDisposed = true;
                GC.SuppressFinalize(this);
            }
        }

        #endregion

        /// <summary>
        /// Resets the dynamics of this body.
        /// Sets torque, force and linear/angular velocity to 0
        /// </summary>
        public void ResetDynamics() {
            Torque = 0;
            AngularVelocityInternal = 0;
            Force = TSVector2.zero;
            LinearVelocityInternal = TSVector2.zero;
        }

        /// <summary>
        /// Creates a fixture and attach it to this body.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// Contacts are not created until the next time step.
        /// Warning: This function is locked during callbacks.
        /// </summary>
        /// <param name="shape">The shape.</param>
        /// <returns></returns>
        public Fixture CreateFixture(Shape shape) {
            return new Fixture(this, shape);
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
        public Fixture CreateFixture(Shape shape, object userData) {
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
        public void DestroyFixture(Fixture fixture) {
            Debug.Assert(fixture.Body == this);

            // Remove the fixture from this body's singly linked list.
            Debug.Assert(FixtureList.Count > 0);

            // You tried to remove a fixture that not present in the fixturelist.
            Debug.Assert(FixtureList.Contains(fixture));

            // Destroy any contacts associated with the fixture.
            ContactEdge edge = ContactList;
            while (edge != null) {
                Contact c = edge.Contact;
                edge = edge.Next;

                Fixture fixtureA = c.FixtureA;
                Fixture fixtureB = c.FixtureB;

                if (fixture == fixtureA || fixture == fixtureB) {
                    // This destroys the contact and removes it from
                    // this body's contact list.
                    World.ContactManager.Destroy(c);
                }
            }

            if ((Flags & BodyFlags.Enabled) == BodyFlags.Enabled) {
                IBroadPhase broadPhase = World.ContactManager.BroadPhase;
                fixture.DestroyProxies(broadPhase);
            }

            FixtureList.Remove(fixture);
            fixture.Destroy();
            fixture.Body = null;

            ResetMassData();
        }

        /// <summary>
        /// Set the position of the body's origin and rotation.
        /// This breaks any contacts and wakes the other bodies.
        /// Manipulating a body's transform may cause non-physical behavior.
        /// </summary>
        /// <param name="position">The world position of the body's local origin.</param>
        /// <param name="rotation">The world rotation in radians.</param>
        public void SetTransform(ref TSVector2 position, FixedPoint rotation) {
            SetTransformIgnoreContacts(ref position, rotation);

            World.ContactManager.FindNewContacts();
        }

        /// <summary>
        /// Set the position of the body's origin and rotation.
        /// This breaks any contacts and wakes the other bodies.
        /// Manipulating a body's transform may cause non-physical behavior.
        /// </summary>
        /// <param name="position">The world position of the body's local origin.</param>
        /// <param name="rotation">The world rotation in radians.</param>
        public void SetTransform(TSVector2 position, FixedPoint rotation) {
            SetTransform(ref position, rotation);
        }

        /// <summary>
        /// For teleporting a body without considering new contacts immediately.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The angle.</param>
        public void SetTransformIgnoreContacts(ref TSVector2 position, FixedPoint angle) {
            Xf.q.Set(angle);
            Xf.p = position;

            Sweep.C = MathUtils.Mul(ref Xf, Sweep.LocalCenter);
            Sweep.A = angle;

            Sweep.C0 = Sweep.C;
            Sweep.A0 = angle;

            IBroadPhase broadPhase = World.ContactManager.BroadPhase;
            for (int i = 0; i < FixtureList.Count; i++) {
                FixtureList[i].Synchronize(broadPhase, ref Xf, ref Xf);
            }
        }

        /// <summary>
        /// Get the body transform for the body's origin.
        /// </summary>
        /// <param name="transform">The transform of the body's origin.</param>
        public void GetTransform(out Transform transform) {
            transform = Xf;
        }

        /// <summary>
        /// Apply a force at a world point. If the force is not
        /// applied at the center of mass, it will generate a torque and
        /// affect the angular velocity. This wakes up the body.
        /// </summary>
        /// <param name="force">The world force vector, usually in Newtons (N).</param>
        /// <param name="point">The world position of the point of application.</param>
        public void ApplyForce(TSVector2 force, TSVector2 point) {
            ApplyForce(ref force, ref point);
        }

        /// <summary>
        /// Applies a force at the center of mass.
        /// </summary>
        /// <param name="force">The force.</param>
        public void ApplyForce(ref TSVector2 force) {
            ApplyForce(ref force, ref Xf.p);
        }

        /// <summary>
        /// Applies a force at the center of mass.
        /// </summary>
        /// <param name="force">The force.</param>
        public void ApplyForce(TSVector2 force) {
            ApplyForce(ref force, ref Xf.p);
        }

        /// <summary>
        /// Apply a force at a world point. If the force is not
        /// applied at the center of mass, it will generate a torque and
        /// affect the angular velocity. This wakes up the body.
        /// </summary>
        /// <param name="force">The world force vector, usually in Newtons (N).</param>
        /// <param name="point">The world position of the point of application.</param>
        public void ApplyForce(ref TSVector2 force, ref TSVector2 point) {
            Debug.Assert(!FixedPoint.IsNaN(force.x));
            Debug.Assert(!FixedPoint.IsNaN(force.y));
            Debug.Assert(!FixedPoint.IsNaN(point.x));
            Debug.Assert(!FixedPoint.IsNaN(point.y));

            if (_bodyType == BodyType.Dynamic) {
                if (Awake == false) {
                    Awake = true;
                }

                Force += force;
                Torque += (point.x - Sweep.C.x) * force.y - (point.y - Sweep.C.y) * force.x;
            }
        }

        /// <summary>
        /// Apply a torque. This affects the angular velocity
        /// without affecting the linear velocity of the center of mass.
        /// This wakes up the body.
        /// </summary>
        /// <param name="torque">The torque about the z-axis (out of the screen), usually in N-m.</param>
        public void ApplyTorque(FixedPoint torque) {
            Debug.Assert(!FixedPoint.IsNaN(torque));

            if (_bodyType == BodyType.Dynamic) {
                if (Awake == false) {
                    Awake = true;
                }

                Torque += torque;
            }
        }

        /// <summary>
        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// This wakes up the body.
        /// </summary>
        /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
        public void ApplyLinearImpulse(TSVector2 impulse) {
            ApplyLinearImpulse(ref impulse);
        }

        /// <summary>
        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass.
        /// This wakes up the body.
        /// </summary>
        /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
        /// <param name="point">The world position of the point of application.</param>
        public void ApplyLinearImpulse(TSVector2 impulse, TSVector2 point) {
            ApplyLinearImpulse(ref impulse, ref point);
        }

        /// <summary>
        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// This wakes up the body.
        /// </summary>
        /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
        public void ApplyLinearImpulse(ref TSVector2 impulse) {
            if (_bodyType != BodyType.Dynamic) {
                return;
            }

            if (Awake == false) {
                Awake = true;
            }

            LinearVelocityInternal += InvMass * impulse;
        }

        /// <summary>
        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass.
        /// This wakes up the body.
        /// </summary>
        /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
        /// <param name="point">The world position of the point of application.</param>
        public void ApplyLinearImpulse(ref TSVector2 impulse, ref TSVector2 point) {
            if (_bodyType != BodyType.Dynamic)
                return;

            if (Awake == false)
                Awake = true;

            LinearVelocityInternal += InvMass * impulse;
            AngularVelocityInternal += InvI * ((point.x - Sweep.C.x) * impulse.y - (point.y - Sweep.C.y) * impulse.x);
        }

        /// <summary>
        /// Apply an angular impulse.
        /// </summary>
        /// <param name="impulse">The angular impulse in units of kg*m*m/s.</param>
        public void ApplyAngularImpulse(FixedPoint impulse) {
            if (_bodyType != BodyType.Dynamic) {
                return;
            }

            if (Awake == false) {
                Awake = true;
            }

            AngularVelocityInternal += InvI * impulse;
        }

        /// <summary>
        /// This resets the mass properties to the sum of the mass properties of the fixtures.
        /// This normally does not need to be called unless you called SetMassData to override
        /// the mass and you later want to reset the mass.
        /// </summary>
        public void ResetMassData() {
            // Compute mass data from shapes. Each shape has its own density.
            _mass = 0.0f;
            InvMass = 0.0f;
            _inertia = 0.0f;
            InvI = 0.0f;
            Sweep.LocalCenter = TSVector2.zero;

            // Kinematic bodies have zero mass.
            if (BodyType == BodyType.Kinematic) {
                Sweep.C0 = Xf.p;
                Sweep.C = Xf.p;
                Sweep.A0 = Sweep.A;
                return;
            }

            Debug.Assert(BodyType == BodyType.Dynamic || BodyType == BodyType.Static);

            // Accumulate mass over all fixtures.
            TSVector2 localCenter = TSVector2.zero;
            foreach (Fixture f in FixtureList) {
                if (f.Shape._density == 0) {
                    continue;
                }

                MassData massData = f.Shape.MassData;
                _mass += massData.Mass;
                localCenter += massData.Mass * massData.Centroid;
                _inertia += massData.Inertia;
            }

            //Static bodies only have mass, they don't have other properties. A little hacky tho...
            if (BodyType == BodyType.Static) {
                Sweep.C0 = Sweep.C = Xf.p;
                return;
            }

            // Compute center of mass.
            if (_mass > 0.0f) {
                InvMass = 1.0f / _mass;
                localCenter *= InvMass;
            }
            else {
                // Force all dynamic bodies to have a positive mass.
                _mass = 1.0f;
                InvMass = 1.0f;
            }

            if (_inertia > 0.0f && (Flags & BodyFlags.FixedRotation) == 0) {
                // Center the inertia about the center of mass.
                _inertia -= _mass * TSVector2.Dot(localCenter, localCenter);

                Debug.Assert(_inertia > 0.0f);
                InvI = 1.0f / _inertia;
            }
            else {
                _inertia = 0.0f;
                InvI = 0.0f;
            }

            // Move center of mass.
            TSVector2 oldCenter = Sweep.C;
            Sweep.LocalCenter = localCenter;
            Sweep.C0 = Sweep.C = MathUtils.Mul(ref Xf, ref Sweep.LocalCenter);

            // Update center of mass velocity.
            TSVector2 a = Sweep.C - oldCenter;
            LinearVelocityInternal += new TSVector2(-AngularVelocityInternal * a.y, AngularVelocityInternal * a.x);
        }

        /// <summary>
        /// Get the world coordinates of a point given the local coordinates.
        /// </summary>
        /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
        /// <returns>The same point expressed in world coordinates.</returns>
        public TSVector2 GetWorldPoint(ref TSVector2 localPoint) {
            return MathUtils.Mul(ref Xf, ref localPoint);
        }

        /// <summary>
        /// Get the world coordinates of a point given the local coordinates.
        /// </summary>
        /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
        /// <returns>The same point expressed in world coordinates.</returns>
        public TSVector2 GetWorldPoint(TSVector2 localPoint) {
            return GetWorldPoint(ref localPoint);
        }

        /// <summary>
        /// Get the world coordinates of a vector given the local coordinates.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="localVector">A vector fixed in the body.</param>
        /// <returns>The same vector expressed in world coordinates.</returns>
        public TSVector2 GetWorldVector(ref TSVector2 localVector) {
            return MathUtils.Mul(Xf.q, localVector);
        }

        /// <summary>
        /// Get the world coordinates of a vector given the local coordinates.
        /// </summary>
        /// <param name="localVector">A vector fixed in the body.</param>
        /// <returns>The same vector expressed in world coordinates.</returns>
        public TSVector2 GetWorldVector(TSVector2 localVector) {
            return GetWorldVector(ref localVector);
        }

        /// <summary>
        /// Gets a local point relative to the body's origin given a world point.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The corresponding local point relative to the body's origin.</returns>
        public TSVector2 GetLocalPoint(ref TSVector2 worldPoint) {
            return MathUtils.MulT(ref Xf, worldPoint);
        }

        /// <summary>
        /// Gets a local point relative to the body's origin given a world point.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The corresponding local point relative to the body's origin.</returns>
        public TSVector2 GetLocalPoint(TSVector2 worldPoint) {
            return GetLocalPoint(ref worldPoint);
        }

        /// <summary>
        /// Gets a local vector given a world vector.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldVector">A vector in world coordinates.</param>
        /// <returns>The corresponding local vector.</returns>
        public TSVector2 GetLocalVector(ref TSVector2 worldVector) {
            return MathUtils.MulT(Xf.q, worldVector);
        }

        /// <summary>
        /// Gets a local vector given a world vector.
        /// Note that the vector only takes the rotation into account, not the position.
        /// </summary>
        /// <param name="worldVector">A vector in world coordinates.</param>
        /// <returns>The corresponding local vector.</returns>
        public TSVector2 GetLocalVector(TSVector2 worldVector) {
            return GetLocalVector(ref worldVector);
        }

        /// <summary>
        /// Get the world linear velocity of a world point attached to this body.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The world velocity of a point.</returns>
        public TSVector2 GetLinearVelocityFromWorldPoint(TSVector2 worldPoint) {
            return GetLinearVelocityFromWorldPoint(ref worldPoint);
        }

        /// <summary>
        /// Get the world linear velocity of a world point attached to this body.
        /// </summary>
        /// <param name="worldPoint">A point in world coordinates.</param>
        /// <returns>The world velocity of a point.</returns>
        public TSVector2 GetLinearVelocityFromWorldPoint(ref TSVector2 worldPoint) {
            return LinearVelocityInternal +
                   new TSVector2(-AngularVelocityInternal * (worldPoint.y - Sweep.C.y),
                       AngularVelocityInternal * (worldPoint.x - Sweep.C.x));
        }

        /// <summary>
        /// Get the world velocity of a local point.
        /// </summary>
        /// <param name="localPoint">A point in local coordinates.</param>
        /// <returns>The world velocity of a point.</returns>
        public TSVector2 GetLinearVelocityFromLocalPoint(TSVector2 localPoint) {
            return GetLinearVelocityFromLocalPoint(ref localPoint);
        }

        /// <summary>
        /// Get the world velocity of a local point.
        /// </summary>
        /// <param name="localPoint">A point in local coordinates.</param>
        /// <returns>The world velocity of a point.</returns>
        public TSVector2 GetLinearVelocityFromLocalPoint(ref TSVector2 localPoint) {
            return GetLinearVelocityFromWorldPoint(GetWorldPoint(ref localPoint));
        }

        public Body DeepClone() {
            Body body = Clone();

            for (int i = 0; i < FixtureList.Count; i++) {
                FixtureList[i].Clone(body);
            }

            return body;
        }

        public Body Clone() {
            Body body = new Body();
            body.World = World;
            body.UserData = UserData;
            body.LinearDamping = LinearDamping;
            body.LinearVelocityInternal = LinearVelocityInternal;
            body.AngularDamping = AngularDamping;
            body.AngularVelocityInternal = AngularVelocityInternal;
            body.Position = Position;
            body.Rotation = Rotation;
            body._bodyType = _bodyType;
            body.Flags = Flags;

            World.AddBody(body);

            return body;
        }

        internal void SynchronizeFixtures() {
            Transform xf1 = new Transform();
            xf1.q.Set(Sweep.A0);
            xf1.p = Sweep.C0 - MathUtils.Mul(xf1.q, Sweep.LocalCenter);

            IBroadPhase broadPhase = World.ContactManager.BroadPhase;
            for (int i = 0; i < FixtureList.Count; i++) {
                FixtureList[i].Synchronize(broadPhase, ref xf1, ref Xf);
            }
        }

        internal void SynchronizeTransform() {
            Xf.q.Set(Sweep.A);
            Xf.p = Sweep.C - MathUtils.Mul(Xf.q, Sweep.LocalCenter);
        }

        /// <summary>
        /// This is used to prevent connected bodies from colliding.
        /// It may lie, depending on the collideConnected flag.
        /// </summary>
        /// <param name="other">The other body.</param>
        /// <returns></returns>
        internal bool ShouldCollide(Body other) {
            // At least one body should be dynamic.
            if (_bodyType != BodyType.Dynamic && other._bodyType != BodyType.Dynamic) {
                return false;
            }

            // Does a joint prevent collision?
            for (JointEdge jn = JointList; jn != null; jn = jn.Next) {
                if (jn.Other == other) {
                    if (jn.Joint.CollideConnected == false) {
                        return false;
                    }
                }
            }

            return true;
        }

        internal void Advance(FixedPoint alpha) {
            // Advance to the new safe time. This doesn't sync the broad-phase.
            Sweep.Advance(alpha);
            Sweep.C = Sweep.C0;
            Sweep.A = Sweep.A0;
            Xf.q.Set(Sweep.A);
            Xf.p = Sweep.C - MathUtils.Mul(Xf.q, Sweep.LocalCenter);
        }

        public event OnCollisionEventHandler OnCollision {
            add {
                for (int i = 0; i < FixtureList.Count; i++) {
                    FixtureList[i].OnCollision += value;
                }
            }
            remove {
                for (int i = 0; i < FixtureList.Count; i++) {
                    FixtureList[i].OnCollision -= value;
                }
            }
        }

        public event OnSeparationEventHandler OnSeparation {
            add {
                for (int i = 0; i < FixtureList.Count; i++) {
                    FixtureList[i].OnSeparation += value;
                }
            }
            remove {
                for (int i = 0; i < FixtureList.Count; i++) {
                    FixtureList[i].OnSeparation -= value;
                }
            }
        }

        public void IgnoreCollisionWith(Body other) {
            for (int i = 0; i < FixtureList.Count; i++) {
                Fixture f = FixtureList[i];
                for (int j = 0; j < other.FixtureList.Count; j++) {
                    Fixture f2 = other.FixtureList[j];

                    f.IgnoreCollisionWith(f2);
                }
            }
        }

        public void RestoreCollisionWith(Body other) {
            for (int i = 0; i < FixtureList.Count; i++) {
                Fixture f = FixtureList[i];
                for (int j = 0; j < other.FixtureList.Count; j++) {
                    Fixture f2 = other.FixtureList[j];

                    f.RestoreCollisionWith(f2);
                }
            }
        }
    }
}