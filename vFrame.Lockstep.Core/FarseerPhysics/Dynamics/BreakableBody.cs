using System.Collections.Generic;
using vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes;
using vFrame.Lockstep.Core.FarseerPhysics.Common;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Contacts;
using vFrame.Lockstep.Core.FarseerPhysics.Factories;

namespace vFrame.Lockstep.Core.FarseerPhysics.Dynamics
{
    /// <summary>
    /// A type of body that supports multiple fixtures that can break apart.
    /// </summary>
    public class BreakableBody
    {
        public bool Broken;
        public Body MainBody;
        public List<Fixture> Parts = new List<Fixture>(8);

        /// <summary>
        /// The force needed to break the body apart.
        /// Default: 500
        /// </summary>
        public FixedPoint Strength = 500.0f;

        private FixedPoint[] _angularVelocitiesCache = new FixedPoint[8];
        private bool _break;
        private TSVector2[] _velocitiesCache = new TSVector2[8];
        private World _world;

        public BreakableBody(IEnumerable<Vertices> vertices, World world, FixedPoint density)
            : this(vertices, world, density, null) {
        }

        public BreakableBody(IEnumerable<Vertices> vertices, World world, FixedPoint density, object userData) {
            _world = world;
            _world.ContactManager.PostSolve += PostSolve;
            MainBody = new Body(_world);
            MainBody.BodyType = BodyType.Dynamic;

            foreach (Vertices part in vertices) {
                PolygonShape polygonShape = new PolygonShape(part, density);
                Fixture fixture = MainBody.CreateFixture(polygonShape, userData);
                Parts.Add(fixture);
            }
        }

        private void PostSolve(Contact contact, ContactVelocityConstraint impulse) {
            if (!Broken) {
                if (Parts.Contains(contact.FixtureA) || Parts.Contains(contact.FixtureB)) {
                    FixedPoint maxImpulse = 0.0f;
                    int count = contact.Manifold.PointCount;

                    for (int i = 0; i < count; ++i) {
                        maxImpulse = TSMath.Max(maxImpulse, impulse.points[i].normalImpulse);
                    }

                    if (maxImpulse > Strength) {
                        // Flag the body for breaking.
                        _break = true;
                    }
                }
            }
        }

        public void Update() {
            if (_break) {
                Decompose();
                Broken = true;
                _break = false;
            }

            // Cache velocities to improve movement on breakage.
            if (Broken == false) {
                //Enlarge the cache if needed
                if (Parts.Count > _angularVelocitiesCache.Length) {
                    _velocitiesCache = new TSVector2[Parts.Count];
                    _angularVelocitiesCache = new FixedPoint[Parts.Count];
                }

                //Cache the linear and angular velocities.
                for (int i = 0; i < Parts.Count; i++) {
                    _velocitiesCache[i] = Parts[i].Body.LinearVelocity;
                    _angularVelocitiesCache[i] = Parts[i].Body.AngularVelocity;
                }
            }
        }

        private void Decompose() {
            //Unsubsribe from the PostSolve delegate
            _world.ContactManager.PostSolve -= PostSolve;

            for (int i = 0; i < Parts.Count; i++) {
                Fixture fixture = Parts[i];

                Shape shape = fixture.Shape.Clone();

                object userdata = fixture.UserData;
                MainBody.DestroyFixture(fixture);

                Body body = BodyFactory.CreateBody(_world);
                body.BodyType = BodyType.Dynamic;
                body.Position = MainBody.Position;
                body.Rotation = MainBody.Rotation;
                body.UserData = MainBody.UserData;

                body.CreateFixture(shape, userdata);

                body.AngularVelocity = _angularVelocitiesCache[i];
                body.LinearVelocity = _velocitiesCache[i];
            }

            _world.RemoveBody(MainBody);
            _world.RemoveBreakableBody(this);
        }

        public void Break() {
            _break = true;
        }
    }
}