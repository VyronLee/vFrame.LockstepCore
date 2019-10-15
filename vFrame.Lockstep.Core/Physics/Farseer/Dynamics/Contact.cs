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
#pragma warning disable 0162

using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// A contact edge is used to connect bodies and contacts together
    /// in a contact graph where each body is a node and each contact
    /// is an edge. A contact edge belongs to a doubly linked list
    /// maintained in each attached body. Each contact has two contact
    /// nodes, one for each attached body.
    /// </summary>
    public sealed class ContactEdge
    {
        /// <summary>
        /// The contact
        /// </summary>
        public Contact Contact;

        /// <summary>
        /// The next contact edge in the body's contact list
        /// </summary>
        public ContactEdge Next;

        /// <summary>
        /// Provides quick access to the other body attached.
        /// </summary>
        public Body Other;

        /// <summary>
        /// The previous contact edge in the body's contact list
        /// </summary>
        public ContactEdge Prev;
    }

    /// <summary>
    /// The class manages contact between two shapes. A contact exists for each overlapping
    /// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
    /// that has no contact points.
    /// </summary>
    public class Contact
    {
        internal ContactType _type;

        private static EdgeShape _edge = new EdgeShape();

        private static ContactType[,] _registers = new[,]
                                                       {
                                                           {
                                                               ContactType.Circle,
                                                               ContactType.EdgeAndCircle,
                                                               ContactType.PolygonAndCircle,
                                                               ContactType.ChainAndCircle,
                                                           },
                                                           {
                                                               ContactType.EdgeAndCircle,
                                                               ContactType.NotSupported,
                                                               // 1,1 is invalid (no ContactType.Edge)
                                                               ContactType.EdgeAndPolygon,
                                                               ContactType.NotSupported,
                                                               // 1,3 is invalid (no ContactType.EdgeAndLoop)
                                                           },
                                                           {
                                                               ContactType.PolygonAndCircle,
                                                               ContactType.EdgeAndPolygon,
                                                               ContactType.Polygon,
                                                               ContactType.ChainAndPolygon,
                                                           },
                                                           {
                                                               ContactType.ChainAndCircle,
                                                               ContactType.NotSupported,
                                                               // 3,1 is invalid (no ContactType.EdgeAndLoop)
                                                               ContactType.ChainAndPolygon,
                                                               ContactType.NotSupported,
                                                               // 3,3 is invalid (no ContactType.Loop)
                                                           },
                                                       };
        // Nodes for connecting bodies.
        internal ContactEdge _nodeA = new ContactEdge();
        internal ContactEdge _nodeB = new ContactEdge();
        internal int _toiCount;
        internal FP _toi;

        public Fixture FixtureA;
        public Fixture FixtureB;
        public FP Friction { get; set; }
        public FP Restitution { get; set; }

        /// <summary>
        /// Get the contact manifold. Do not modify the manifold unless you understand the
        /// internals of Box2D.
        /// </summary>
        public Manifold Manifold;

        internal ContactCloneKey Key {
            get {
                return new ContactCloneKey(FixtureA.FixtureId, FixtureB.FixtureId);
            }
        }

        /// Get or set the desired tangent speed for a conveyor belt behavior. In meters per second.
        public FP TangentSpeed { get; set; }

        /// Enable/disable this contact. This can be used inside the pre-solve
        /// contact listener. The contact is only disabled for the current
        /// time step (or sub-step in continuous collisions).
        /// NOTE: If you are setting Enabled to a constant true or false,
        /// use the explicit Enable() or Disable() functions instead to 
        /// save the CPU from doing a branch operation.
        public bool Enabled { get; set; }

        /// <summary>
        /// Get the child primitive index for fixture A.
        /// </summary>
        /// <value>The child index A.</value>
        public int ChildIndexA { get; internal set; }

        /// <summary>
        /// Get the child primitive index for fixture B.
        /// </summary>
        /// <value>The child index B.</value>
        public int ChildIndexB { get; internal set; }

        /// <summary>
        /// Determines whether this contact is touching.
        /// </summary>
        /// <returns>
        /// 	<c>true</c> if this instance is touching; otherwise, <c>false</c>.
        /// </returns>
        public bool IsTouching { get; set; }

        internal bool IslandFlag { get; set; }
        internal bool TOIFlag { get; set; }
        internal bool FilterFlag { get; set; }

        public void ResetRestitution()
        {
            Restitution = Settings.MixRestitution(FixtureA.Restitution, FixtureB.Restitution);
        }

        public void ResetFriction()
        {
            Friction = Settings.MixFriction(FixtureA.Friction, FixtureB.Friction);
        }

        internal Contact() {}

        internal Contact(World world, Fixture fA, int indexA, Fixture fB, int indexB)
        {
            m_world = world;
            Reset(fA, indexA, fB, indexB);
        }
        
        private void Reset(Fixture fA, int indexA, Fixture fB, int indexB)
        {
            Enabled = true;
            IsTouching = false;
            IslandFlag = false;
            FilterFlag = false;
            TOIFlag = false;

            FixtureA = fA;
            FixtureB = fB;

            ChildIndexA = indexA;
            ChildIndexB = indexB;

            Manifold.PointCount = 0;

            _nodeA.Contact = null;
            _nodeA.Prev = null;
            _nodeA.Next = null;
            _nodeA.Other = null;

            _nodeB.Contact = null;
            _nodeB.Prev = null;
            _nodeB.Next = null;
            _nodeB.Other = null;

            _toiCount = 0;

            //FPE: We only set the friction and restitution if we are not destroying the contact
            if (FixtureA != null && FixtureB != null)
            {
                Friction = Settings.MixFriction(FixtureA.Friction, FixtureB.Friction);
                Restitution = Settings.MixRestitution(FixtureA.Restitution, FixtureB.Restitution);
            }

            TangentSpeed = 0;
        }

        /// <summary>
        /// Evaluate this contact with your own manifold and transforms.   
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="transformA">The first transform.</param>
        /// <param name="transformB">The second transform.</param>
        internal void Evaluate(ref Manifold manifold, ref Transform transformA, ref Transform transformB)
        {
            switch (_type)
            {
                case ContactType.Polygon:
                    Collision.CollidePolygons(ref manifold, (PolygonShape)FixtureA.Shape, ref transformA, (PolygonShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.PolygonAndCircle:
                    Collision.CollidePolygonAndCircle(ref manifold, (PolygonShape)FixtureA.Shape, ref transformA, (CircleShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.EdgeAndCircle:
                    Collision.CollideEdgeAndCircle(ref manifold, (EdgeShape)FixtureA.Shape, ref transformA, (CircleShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.EdgeAndPolygon:
                    Collision.CollideEdgeAndPolygon(ref manifold, (EdgeShape)FixtureA.Shape, ref transformA, (PolygonShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.ChainAndCircle:
                    ChainShape chain = (ChainShape)FixtureA.Shape;
                    chain.GetChildEdge(_edge, ChildIndexA);
                    Collision.CollideEdgeAndCircle(ref manifold, _edge, ref transformA, (CircleShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.ChainAndPolygon:
                    ChainShape loop2 = (ChainShape)FixtureA.Shape;
                    loop2.GetChildEdge(_edge, ChildIndexA);
                    Collision.CollideEdgeAndPolygon(ref manifold, _edge, ref transformA, (PolygonShape)FixtureB.Shape, ref transformB);
                    break;
                case ContactType.Circle:
                    Collision.CollideCircles(ref manifold, (CircleShape)FixtureA.Shape, ref transformA, (CircleShape)FixtureB.Shape, ref transformB);
                    break;
            }
        }

        internal static Contact Create(World world, Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            ShapeType type1 = fixtureA.Shape.ShapeType;
            ShapeType type2 = fixtureB.Shape.ShapeType;

            Debug.Assert(ShapeType.Unknown < type1 && type1 < ShapeType.TypeCount);
            Debug.Assert(ShapeType.Unknown < type2 && type2 < ShapeType.TypeCount);

            Contact c;
            Queue<Contact> pool = world._contactPool;
            if (pool.Count > 0)
            {
                c = pool.Dequeue();
                if ((type1 >= type2 || (type1 == ShapeType.Edge && type2 == ShapeType.Polygon)) && !(type2 == ShapeType.Edge && type1 == ShapeType.Polygon))
                {
                    c.Reset(fixtureA, indexA, fixtureB, indexB);
                }
                else
                {
                    c.Reset(fixtureB, indexB, fixtureA, indexA);
                }
            }
            else
            {
                // Edge+Polygon is non-symetrical due to the way Erin handles collision type registration.
                if ((type1 >= type2 || (type1 == ShapeType.Edge && type2 == ShapeType.Polygon)) && !(type2 == ShapeType.Edge && type1 == ShapeType.Polygon))
                {
                    c = new Contact(world, fixtureA, indexA, fixtureB, indexB);
                }
                else
                {
                    c = new Contact(world, fixtureB, indexB, fixtureA, indexA);
                }
            }

            c._type = _registers[(int)type1, (int)type2];

            return c;
        }

        internal void Destroy()
        {
#if USE_ACTIVE_CONTACT_SET
            FixtureA.Body.World.ContactManager.RemoveActiveContact(this);
#endif
            m_world._contactPool.Enqueue(this);

            if (Manifold.PointCount > 0 && FixtureA.IsSensor == false && FixtureB.IsSensor == false)
            {
                FixtureA.Body.Awake = true;
                FixtureB.Body.Awake = true;
            }

            Reset(null, 0, null, 0);
        }

        private World m_world;

        #region Nested type: ContactType

        internal enum ContactType
        {
            NotSupported,
            Polygon,
            PolygonAndCircle,
            Circle,
            EdgeAndPolygon,
            EdgeAndCircle,
            ChainAndPolygon,
            ChainAndCircle,
        }

        #endregion
    }
}