using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.FarseerPhysics;
using vFrame.Lockstep.Core.FarseerPhysics.Collision;
using vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes;
using vFrame.Lockstep.Core.FarseerPhysics.Common;
using vFrame.Lockstep.Core.FarseerPhysics.Controllers;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Contacts;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints;
//using Microsoft.Xna.Framework.Content;
//using Microsoft.Xna.Framework.Graphics;
using GL = UnityEngine.GL;
using Color = UnityEngine.Color;
using Vector3 = UnityEngine.Vector3;
using Rectangle = UnityEngine.Rect;
using Gizmos = UnityEngine.Gizmos;

namespace FarseerPhysics.DebugViews
{
    /// <summary>
    /// A debug view that works in Unity. #gabshack
    /// A debug view shows you what happens inside the physics engine. You can view
    /// bodies, joints, fixtures and more.
    /// </summary>
    public class FarseerDebugViewUnity : DebugView, IDisposable
    {
        //Drawing
        //private PrimitiveBatch _primitiveBatch;
		private List<FDVertex> lineListBatch;
		private List<FDVertex> triangleListBatch;
		//private FDVertex[] triangleListBatch;
        //private SpriteBatch _batch;
        //private SpriteFont _font;
        //private GraphicsDevice _device;
        private TSVector2[] _tempVertices = new TSVector2[Settings.MaxPolygonVertices];
        private List<StringData> _stringData;

        private TSMatrix4x4 _localProjection;
        private TSMatrix4x4 _localView;

        //Shapes
        public Color DefaultShapeColor = new Color(0.9f, 0.7f, 0.7f);
        public Color InactiveShapeColor = new Color(0.5f, 0.5f, 0.3f);
        public Color KinematicShapeColor = new Color(0.5f, 0.5f, 0.9f);
        public Color SleepingShapeColor = new Color(0.6f, 0.6f, 0.6f);
        public Color StaticShapeColor = new Color(0.5f, 0.9f, 0.5f);
        public Color TextColor = Color.white;

        //Contacts
        private int _pointCount;
        private const int MaxContactPoints = 2048;
        private ContactPoint[] _points = new ContactPoint[MaxContactPoints];

        //Debug panel
#if XBOX
        public TSVector2 DebugPanelPosition = new TSVector2(55, 100);
#else
        public TSVector2 DebugPanelPosition = new TSVector2(400, 300);
#endif
        private int _max;
        private int _avg;
        private int _min;

        //Performance graph
        public bool AdaptiveLimits = true;
        public int ValuesToGraph = 500;
        public int MinimumValue;
        public int MaximumValue = 1000;
        private List<float> _graphValues = new List<float>();

        public Rectangle PerformancePanelBounds = new Rectangle(35f, 13f, 8f, 4f);

        private TSVector2[] _background = new TSVector2[4];
        public bool Enabled = true;

#if XBOX || WINDOWS_PHONE
        public const int CircleSegments = 16;
#else
        public const int CircleSegments = 32;
#endif

        public FarseerDebugViewUnity(World world)
            : base(world)
        {
			lineListBatch = new List<FDVertex>();
			triangleListBatch = new List<FDVertex>();
            world.ContactManager.PreSolve += PreSolve;

            //Default flags
            AppendFlags(DebugViewFlags.Shape);
            AppendFlags(DebugViewFlags.Controllers);
            AppendFlags(DebugViewFlags.Joint);
        }

        public void BeginCustomDraw(ref TSMatrix4x4 projection, ref TSMatrix4x4 view)
        {
            //_primitiveBatch.Begin(ref projection, ref view);
        }

        public void EndCustomDraw()
        {
            //_primitiveBatch.End();
        }

        #region IDisposable Members

        public void Dispose()
        {
            World.ContactManager.PreSolve -= PreSolve;
        }

        #endregion

        private void PreSolve(Contact contact, ref Manifold oldManifold)
        {
            if ((Flags & DebugViewFlags.ContactPoints) == DebugViewFlags.ContactPoints)
            {
                Manifold manifold = contact.Manifold;

                if (manifold.PointCount == 0)
                {
                    return;
                }

                Fixture fixtureA = contact.FixtureA;

                FixedArray2<PointState> state1, state2;
                Collision.GetPointStates(out state1, out state2, ref oldManifold, ref manifold);

                FixedArray2<TSVector2> points;
                TSVector2 normal;
                contact.GetWorldManifold(out normal, out points);

                for (int i = 0; i < manifold.PointCount && _pointCount < MaxContactPoints; ++i)
                {
                    if (fixtureA == null)
                    {
                        _points[i] = new ContactPoint();
                    }
                    ContactPoint cp = _points[_pointCount];
                    cp.Position = points[i];
                    cp.Normal = normal;
                    cp.State = state2[i];
                    _points[_pointCount] = cp;
                    ++_pointCount;
                }
            }
        }

        /// <summary>
        /// Call this to draw shapes and other debug draw data.
        /// </summary>
        private void DrawDebugData()
        {
            if ((Flags & DebugViewFlags.Shape) == DebugViewFlags.Shape)
            {
                foreach (Body b in World.BodyList)
                {
                    Transform xf;
                    b.GetTransform(out xf);
                    foreach (Fixture f in b.FixtureList)
                    {
                        if (b.Enabled == false)
                        {
                            DrawShape(f, xf, InactiveShapeColor);
                        }
                        else if (b.BodyType == BodyType.Static)
                        {
                            DrawShape(f, xf, StaticShapeColor);
                        }
                        else if (b.BodyType == BodyType.Kinematic)
                        {
                            DrawShape(f, xf, KinematicShapeColor);
                        }
                        else if (b.Awake == false)
                        {
                            DrawShape(f, xf, SleepingShapeColor);
                        }
                        else
                        {
                            DrawShape(f, xf, DefaultShapeColor);
                        }
                    }
                }
            }
            if ((Flags & DebugViewFlags.ContactPoints) == DebugViewFlags.ContactPoints)
            {
                float axisScale = 0.3f;

                for (int i = 0; i < _pointCount; ++i)
                {
                    ContactPoint point = _points[i];

                    if (point.State == PointState.Add)
                    {
                        // Add
                        DrawPoint(point.Position, 0.1f, new Color(0.3f, 0.95f, 0.3f));
                    }
                    else if (point.State == PointState.Persist)
                    {
                        // Persist
                        DrawPoint(point.Position, 0.1f, new Color(0.3f, 0.3f, 0.95f));
                    }

                    if ((Flags & DebugViewFlags.ContactNormals) == DebugViewFlags.ContactNormals)
                    {
                        TSVector2 p1 = point.Position;
                        TSVector2 p2 = p1 + axisScale * point.Normal;
                        DrawSegment(p1, p2, new Color(0.4f, 0.9f, 0.4f));
                    }
                }
                _pointCount = 0;
            }
            if ((Flags & DebugViewFlags.PolygonPoints) == DebugViewFlags.PolygonPoints)
            {
                foreach (Body body in World.BodyList)
                {
                    foreach (Fixture f in body.FixtureList)
                    {
                        PolygonShape polygon = f.Shape as PolygonShape;
                        if (polygon != null)
                        {
                            Transform xf;
                            body.GetTransform(out xf);

                            for (int i = 0; i < polygon.Vertices.Count; i++)
                            {
                                TSVector2 tmp = MathUtils.Mul(ref xf, polygon.Vertices[i]);
                                DrawPoint(tmp, 0.1f, Color.red);
                            }
                        }
                    }
                }
            }
            if ((Flags & DebugViewFlags.Joint) == DebugViewFlags.Joint)
            {
                foreach (FarseerJoint j in World.JointList)
                {
                    DrawJoint(j);
                }
            }
            if ((Flags & DebugViewFlags.Pair) == DebugViewFlags.Pair)
            {
                Color color = new Color(0.3f, 0.9f, 0.9f);
                for (int i = 0; i < World.ContactManager.ContactList.Count; i++)
                {
                    Contact c = World.ContactManager.ContactList[i];
                    Fixture fixtureA = c.FixtureA;
                    Fixture fixtureB = c.FixtureB;

                    AABB aabbA;
                    fixtureA.GetAABB(out aabbA, 0);
                    AABB aabbB;
                    fixtureB.GetAABB(out aabbB, 0);

                    TSVector2 cA = aabbA.Center;
                    TSVector2 cB = aabbB.Center;

                    DrawSegment(cA, cB, color);
                }
            }
            if ((Flags & DebugViewFlags.AABB) == DebugViewFlags.AABB)
            {
                Color color = new Color(0.9f, 0.3f, 0.9f);
                IBroadPhase bp = World.ContactManager.BroadPhase;

                foreach (Body b in World.BodyList)
                {
                    if (b.Enabled == false)
                    {
                        continue;
                    }

                    foreach (Fixture f in b.FixtureList)
                    {
                        for (int t = 0; t < f.ProxyCount; ++t)
                        {
                            FixtureProxy proxy = f.Proxies[t];
                            AABB aabb;
                            bp.GetFatAABB(proxy.ProxyId, out aabb);

                            DrawAABB(ref aabb, color);
                        }
                    }
                }
            }
            if ((Flags & DebugViewFlags.CenterOfMass) == DebugViewFlags.CenterOfMass)
            {
                foreach (Body b in World.BodyList)
                {
                    Transform xf;
                    b.GetTransform(out xf);
                    xf.p = b.WorldCenter;
                    DrawTransform(ref xf);
                }
            }
            if ((Flags & DebugViewFlags.Controllers) == DebugViewFlags.Controllers)
            {
                for (int i = 0; i < World.ControllerList.Count; i++)
                {
                    Controller controller = World.ControllerList[i];

                    BuoyancyController buoyancy = controller as BuoyancyController;
                    if (buoyancy != null)
                    {
                        AABB container = buoyancy.Container;
                        DrawAABB(ref container, new Color(0.3f, 0.5f, 1.0f));
                    }
                }
            }

        }

		public void OnGUI(UnityEngine.Camera camera)
		{
			if ((Flags & DebugViewFlags.DebugPanel) == DebugViewFlags.DebugPanel)
            {
                DrawDebugPanel(camera);
            }
		}

        private void DrawPerformanceGraph()
        {
            _graphValues.Add((float)World.UpdateTime);

            if (_graphValues.Count > ValuesToGraph + 1)
                _graphValues.RemoveAt(0);

            float x = PerformancePanelBounds.x;
            float deltaX = PerformancePanelBounds.width / (float)ValuesToGraph;
            float yScale = PerformancePanelBounds.yMax - (float)PerformancePanelBounds.yMin;

            // we must have at least 2 values to start rendering
            if (_graphValues.Count > 2)
            {
                _max = (int)_graphValues.Max();
                _avg = (int)_graphValues.Average();
                _min = (int)_graphValues.Min();

                if (AdaptiveLimits)
                {
                    MaximumValue = _max;
                    MinimumValue = 0;
                }

                // start at last value (newest value added)
                // continue until no values are left
                for (int i = _graphValues.Count - 1; i > 0; i--)
                {
                    float y1 = PerformancePanelBounds.yMax -
                               ((_graphValues[i] / (MaximumValue - MinimumValue)) * yScale);
                    float y2 = PerformancePanelBounds.yMax -
                               ((_graphValues[i - 1] / (MaximumValue - MinimumValue)) * yScale);

                    TSVector2 x1 =
                        new TSVector2(MathHelper.Clamp(x, PerformancePanelBounds.xMin, PerformancePanelBounds.xMax),
                                    MathHelper.Clamp(y1, PerformancePanelBounds.yMin, PerformancePanelBounds.yMax));

                    TSVector2 x2 =
                        new TSVector2(
                            MathHelper.Clamp(x + deltaX, PerformancePanelBounds.xMin, PerformancePanelBounds.xMax),
                            MathHelper.Clamp(y2, PerformancePanelBounds.yMin, PerformancePanelBounds.yMax));

                    DrawSegment(x1, x2, new Color(0.5f, 1.0f, 0.3f));

                    x += deltaX;
                }
            }

            DrawString((int)PerformancePanelBounds.xMax + 10, (int)PerformancePanelBounds.yMin, "Max: " + _max.ToString());
            DrawString((int)PerformancePanelBounds.xMax + 10, (int)PerformancePanelBounds.yMin + (int)PerformancePanelBounds.height/2 - 7, "Avg: " + _avg.ToString());
            DrawString((int)PerformancePanelBounds.xMax + 10, (int)PerformancePanelBounds.yMax - 15, "Min: " + _min.ToString());

            //Draw background.
            _background[0] = new TSVector2(PerformancePanelBounds.x, PerformancePanelBounds.y);
            _background[1] = new TSVector2(PerformancePanelBounds.x,
                                         PerformancePanelBounds.y + PerformancePanelBounds.height);
            _background[2] = new TSVector2(PerformancePanelBounds.x + PerformancePanelBounds.width,
                                         PerformancePanelBounds.y + PerformancePanelBounds.height);
            _background[3] = new TSVector2(PerformancePanelBounds.x + PerformancePanelBounds.width,
                                         PerformancePanelBounds.y);

            DrawSolidPolygon(_background, 4, new Color(0.3f, 0.3f, 0.3f), true);
        }

        private void DrawDebugPanel(UnityEngine.Camera camera)
        {
            int fixtures = 0;
            for (int i = 0; i < World.BodyList.Count; i++)
            {
                fixtures += World.BodyList[i].FixtureList.Count;
            }

            int x = 0;
            int y = 0;

			Vector3 in0 = camera.ScreenToWorldPoint(new Vector3(camera.pixelWidth - 210f, camera.pixelHeight - 120f, 0f));
			//PerformancePanelBounds.x = in0.x / 3f;
			//PerformancePanelBounds.y = (in0.y - 120f) / 3f;

			UnityEngine.GUI.BeginGroup(new UnityEngine.Rect(in0.x, in0.y, 210f, 120f), UnityEngine.GUI.skin.box);

            DrawString(x, y, "Objects:" +
                             "\n- Bodies: " + World.BodyList.Count +
                             "\n- Fixtures: " + fixtures +
                             "\n- Contacts: " + World.ContactList.Count +
                             "\n- Joints: " + World.JointList.Count +
                             "\n- Controllers: " + World.ControllerList.Count +
                             "\n- Proxies: " + World.ProxyCount);

            DrawString(x + 110, y, "Update time:" +
                                   "\n- Body: " + World.SolveUpdateTime +
                                   "\n- Contact: " + World.ContactsUpdateTime +
                                   "\n- CCD: " + World.ContinuousPhysicsTime +
                                   "\n- Joint: " + World.Island.JointUpdateTime +
                                   "\n- Controller: " + World.ControllersUpdateTime +
                                   "\n- Total: " + World.UpdateTime);

			UnityEngine.GUI.EndGroup();
        }

        public void DrawAABB(ref AABB aabb, Color color)
        {
            TSVector2[] verts = new TSVector2[4];
            verts[0] = new TSVector2(aabb.LowerBound.x, aabb.LowerBound.y);
            verts[1] = new TSVector2(aabb.UpperBound.x, aabb.LowerBound.y);
            verts[2] = new TSVector2(aabb.UpperBound.x, aabb.UpperBound.y);
            verts[3] = new TSVector2(aabb.LowerBound.x, aabb.UpperBound.y);

            DrawPolygon(verts, 4, color);
        }

        private void DrawJoint(FarseerJoint joint)
        {
            if (!joint.Enabled)
                return;

            Body b1 = joint.BodyA;
            Body b2 = joint.BodyB;
            Transform xf1, xf2;
            b1.GetTransform(out xf1);

            TSVector2 x2 = TSVector2.zero;

            // WIP David
            if (!joint.IsFixedType())
            {
                b2.GetTransform(out xf2);
                x2 = xf2.p;
            }

            TSVector2 p1 = joint.WorldAnchorA;
            TSVector2 p2 = joint.WorldAnchorB;
            TSVector2 x1 = xf1.p;

            Color color = new Color(0.5f, 0.8f, 0.8f);

            switch (joint.JointType)
            {
                case JointType.Distance:
                    DrawSegment(p1, p2, color);
                    break;
                case JointType.Pulley:
                    PulleyJoint pulley = (PulleyJoint)joint;
                    TSVector2 s1 = pulley.GroundAnchorA;
                    TSVector2 s2 = pulley.GroundAnchorB;
                    DrawSegment(s1, p1, color);
                    DrawSegment(s2, p2, color);
                    DrawSegment(s1, s2, color);
                    break;
                case JointType.FixedMouse:
                    DrawPoint(p1, 0.5f, new Color(0.0f, 1.0f, 0.0f));
                    DrawSegment(p1, p2, new Color(0.8f, 0.8f, 0.8f));
                    break;
                case JointType.Revolute:
                    //DrawSegment(x2, p1, color);
                    DrawSegment(p2, p1, color);
                    DrawSolidCircle(p2, 0.1f, TSVector2.zero, Color.red);
                    DrawSolidCircle(p1, 0.1f, TSVector2.zero, Color.blue);
                    break;
                case JointType.FixedAngle:
                    //Should not draw anything.
                    break;
                case JointType.FixedRevolute:
                    DrawSegment(x1, p1, color);
                    DrawSolidCircle(p1, 0.1f, TSVector2.zero, new Color(1f, 0.5f, 1f));
                    break;
                case JointType.FixedLine:
                    DrawSegment(x1, p1, color);
                    DrawSegment(p1, p2, color);
                    break;
                case JointType.FixedDistance:
                    DrawSegment(x1, p1, color);
                    DrawSegment(p1, p2, color);
                    break;
                case JointType.FixedPrismatic:
                    DrawSegment(x1, p1, color);
                    DrawSegment(p1, p2, color);
                    break;
                case JointType.Gear:
                    DrawSegment(x1, x2, color);
                    break;
                //case JointType.Weld:
                //    break;
                default:
                    DrawSegment(x1, p1, color);
                    DrawSegment(p1, p2, color);
                    DrawSegment(x2, p2, color);
                    break;
            }
        }

        public void DrawShape(Fixture fixture, Transform xf, Color color)
        {
            switch (fixture.ShapeType)
            {
                case ShapeType.Circle:
                    {
                        CircleShape circle = (CircleShape)fixture.Shape;

                        TSVector2 center = MathUtils.Mul(ref xf, circle.Position);
                        float radius = (float)circle.Radius;
                        TSVector2 axis = MathUtils.Mul(xf.q, new TSVector2(1.0f, 0.0f));

                        DrawSolidCircle(center, radius, axis, color);
                    }
                    break;

                case ShapeType.Polygon:
                    {
                        PolygonShape poly = (PolygonShape)fixture.Shape;
                        int vertexCount = poly.Vertices.Count;
                        Debug.Assert(vertexCount <= Settings.MaxPolygonVertices);

                        for (int i = 0; i < vertexCount; ++i)
                        {
                            _tempVertices[i] = MathUtils.Mul(ref xf, poly.Vertices[i]);
                        }

                        DrawSolidPolygon(_tempVertices, vertexCount, color);
                    }
                    break;


                case ShapeType.Edge:
                    {
                        EdgeShape edge = (EdgeShape)fixture.Shape;
                        TSVector2 v1 = MathUtils.Mul(ref xf, edge.Vertex1);
                        TSVector2 v2 = MathUtils.Mul(ref xf, edge.Vertex2);
                        DrawSegment(v1, v2, color);
                    }
                    break;

                case ShapeType.Chain:
                    {
                        ChainShape chain = (ChainShape)fixture.Shape;
                        int count = chain.Vertices.Count;

                        TSVector2 v1 = MathUtils.Mul(ref xf, chain.Vertices[count - 1]);
                        DrawCircle(v1, 0.05f, color);
                        for (int i = 0; i < count; ++i)
                        {
                            TSVector2 v2 = MathUtils.Mul(ref xf, chain.Vertices[i]);
                            DrawSegment(v1, v2, color);
                            v1 = v2;
                        }
                    }
                    break;
            }
        }

        public override void DrawPolygon(TSVector2[] vertices, int count, FixedPoint red, FixedPoint green, FixedPoint blue)
        {
            DrawPolygon(vertices, count, new Color((float)red, (float)green, (float)blue));
        }

        public void DrawPolygon(TSVector2[] vertices, int count, Color color)
        {
            //if (!_primitiveBatch.IsReady())
            //{
            //    throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
            //}
            for (int i = 0; i < count - 1; i++)
            {
				lineListBatch.Add(FDVertex.FromLine(vertices[i], vertices[i + 1], color));
                //_primitiveBatch.AddVertex(vertices[i], color, PrimitiveType.LineList);
                //_primitiveBatch.AddVertex(vertices[i + 1], color, PrimitiveType.LineList);
            }

			lineListBatch.Add(FDVertex.FromLine(vertices[count - 1], vertices[0], color));
            //_primitiveBatch.AddVertex(vertices[count - 1], color, PrimitiveType.LineList);
            //_primitiveBatch.AddVertex(vertices[0], color, PrimitiveType.LineList);
        }

        public override void DrawSolidPolygon(TSVector2[] vertices, int count, FixedPoint red, FixedPoint green, FixedPoint blue)
        {
            DrawSolidPolygon(vertices, count, new Color((float)red, (float)green, (float)blue), true);
        }

        public void DrawSolidPolygon(TSVector2[] vertices, int count, Color color)
        {
            DrawSolidPolygon(vertices, count, color, true);
        }

        public void DrawSolidPolygon(TSVector2[] vertices, int count, Color color, bool outline)
        {
            //if (!_primitiveBatch.IsReady())
            //{
            //    throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
            //}
            if (count == 2)
            {
                DrawPolygon(vertices, count, color);
                return;
            }

            Color colorFill = color * (outline ? 0.5f : 1.0f);

            for (int i = 1; i < count - 1; i++)
            {
				triangleListBatch.Add(FDVertex.FromTriangleList(vertices[0], vertices[i], vertices[i + 1], colorFill));
                //_primitiveBatch.AddVertex(vertices[0], colorFill, PrimitiveType.TriangleList);
                //_primitiveBatch.AddVertex(vertices[i], colorFill, PrimitiveType.TriangleList);
                //_primitiveBatch.AddVertex(vertices[i + 1], colorFill, PrimitiveType.TriangleList);
            }

            if (outline)
            {
                DrawPolygon(vertices, count, color);
            }
        }

        public override void DrawCircle(TSVector2 center, FixedPoint radius, FixedPoint red, FixedPoint green, FixedPoint blue)
        {
            DrawCircle(center, (float)radius, new Color((float)red, (float)green, (float)blue));
        }

        public void DrawCircle(TSVector2 center, float radius, Color color)
        {
            //if (!_primitiveBatch.IsReady())
            //{
            //    throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
            //}
            const double increment = Math.PI * 2.0 / CircleSegments;
            double theta = 0.0;

            for (int i = 0; i < CircleSegments; i++)
            {
                TSVector2 v1 = center + radius * new TSVector2((float)Math.Cos(theta), (float)Math.Sin(theta));
                TSVector2 v2 = center +
                             radius *
                             new TSVector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

				lineListBatch.Add(FDVertex.FromLine(v1, v2, color));
                //_primitiveBatch.AddVertex(v1, color, PrimitiveType.LineList);
                //_primitiveBatch.AddVertex(v2, color, PrimitiveType.LineList);

                theta += increment;
            }
        }

        public override void DrawSolidCircle(TSVector2 center, FixedPoint radius, TSVector2 axis, FixedPoint red, FixedPoint green,
                                             FixedPoint blue)
        {
            DrawSolidCircle(center, (float)radius, axis, new Color((float)red, (float)green, (float)blue));
        }

        public void DrawSolidCircle(TSVector2 center, float radius, TSVector2 axis, Color color)
        {
            //if (!_primitiveBatch.IsReady())
            //{
            //    throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
            //}
            const double increment = Math.PI * 2.0 / CircleSegments;
            double theta = 0.0;

            Color colorFill = color * 0.5f;

            TSVector2 v0 = center + radius * new TSVector2((float)Math.Cos(theta), (float)Math.Sin(theta));
            theta += increment;

            for (int i = 1; i < CircleSegments - 1; i++)
            {
                TSVector2 v1 = center + radius * new TSVector2((float)Math.Cos(theta), (float)Math.Sin(theta));
                TSVector2 v2 = center +
                             radius *
                             new TSVector2((float)Math.Cos(theta + increment), (float)Math.Sin(theta + increment));

				triangleListBatch.Add(FDVertex.FromTriangleList(v0, v1, v2, colorFill));
                //_primitiveBatch.AddVertex(v0, colorFill, PrimitiveType.TriangleList);
                //_primitiveBatch.AddVertex(v1, colorFill, PrimitiveType.TriangleList);
                //_primitiveBatch.AddVertex(v2, colorFill, PrimitiveType.TriangleList);

                theta += increment;
            }
            DrawCircle(center, radius, color);

            DrawSegment(center, center + axis * radius, color);
        }

        public override void DrawSegment(TSVector2 start, TSVector2 end, FixedPoint red, FixedPoint green, FixedPoint blue)
        {
            DrawSegment(start, end, new Color((float)red, (float)green, (float)blue));
        }

        public void DrawSegment(TSVector2 start, TSVector2 end, Color color)
        {
            //if (!_primitiveBatch.IsReady())
            //{
            //    throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
            //}
			lineListBatch.Add(FDVertex.FromLine(start, end, color));
            //_primitiveBatch.AddVertex(start, color, PrimitiveType.LineList);
            //_primitiveBatch.AddVertex(end, color, PrimitiveType.LineList);
        }

        public override void DrawTransform(ref Transform transform)
        {
            const float axisScale = 0.4f;
            TSVector2 p1 = transform.p;

            TSVector2 p2 = p1 + axisScale * transform.q.GetXAxis();
            DrawSegment(p1, p2, Color.red);

            p2 = p1 + axisScale * transform.q.GetYAxis();
            DrawSegment(p1, p2, Color.green);
        }

        public void DrawPoint(TSVector2 p, float size, Color color)
        {
            TSVector2[] verts = new TSVector2[4];
            float hs = size / 2.0f;
            verts[0] = p + new TSVector2(-hs, -hs);
            verts[1] = p + new TSVector2(hs, -hs);
            verts[2] = p + new TSVector2(hs, hs);
            verts[3] = p + new TSVector2(-hs, hs);

            DrawSolidPolygon(verts, 4, color, true);
        }

        public void DrawString(int x, int y, string s, params object[] args)
        {
			//UnityEngine.GUI.color = TextColor;
			UnityEngine.GUI.Label(new UnityEngine.Rect(x, y, s.Length * 15f, 120f), s);
            //_stringData.Add(new StringData(x, y, s, args, TextColor));
        }

        public void DrawArrow(TSVector2 start, TSVector2 end, float length, float width, bool drawStartIndicator,
                              Color color)
        {
            // Draw connection segment between start- and end-point
            DrawSegment(start, end, color);

            // Precalculate halfwidth
            float halfWidth = width / 2;

            // Create directional reference
            TSVector2 rotation = (start - end);
            rotation.Normalize();

            // Calculate angle of directional vector
            float angle = (float)Math.Atan2((double)rotation.x, (double)-rotation.y);
            // Create matrix for rotation
            TSMatrix4x4 rotFMatrix = TSMatrix4x4.RotateZ(angle);
            // Create translation matrix for end-point
            TSMatrix4x4 endFMatrix = TSMatrix4x4.Translate(end.x, end.y, 0);

            // Setup arrow end shape
            TSVector2[] verts = new TSVector2[3];
            verts[0] = new TSVector2(0, 0);
            verts[1] = new TSVector2(-halfWidth, -length);
            verts[2] = new TSVector2(halfWidth, -length);

            // Rotate end shape
            TSVector2.Transform(verts, ref rotFMatrix, verts);
            // Translate end shape
            TSVector2.Transform(verts, ref endFMatrix, verts);

            // Draw arrow end shape
            DrawSolidPolygon(verts, 3, color, false);

            if (drawStartIndicator)
            {
                // Create translation matrix for start
                TSMatrix4x4 startFMatrix = TSMatrix4x4.Translate(start.x, start.y, 0);
                // Setup arrow start shape
                TSVector2[] baseVerts = new TSVector2[4];
                baseVerts[0] = new TSVector2(-halfWidth, length / 4);
                baseVerts[1] = new TSVector2(halfWidth, length / 4);
                baseVerts[2] = new TSVector2(halfWidth, 0);
                baseVerts[3] = new TSVector2(-halfWidth, 0);

                // Rotate start shape
                TSVector2.Transform(baseVerts, ref rotFMatrix, baseVerts);
                // Translate start shape
                TSVector2.Transform(baseVerts, ref startFMatrix, baseVerts);
                // Draw start shape
                DrawSolidPolygon(baseVerts, 4, color, false);
            }
        }

        public void RenderDebugData(ref UnityEngine.Matrix4x4 projection, ref UnityEngine.Matrix4x4 view, UnityEngine.Material mat)
        {
            if (!Enabled)
            {
                return;
            }

            //Nothing is enabled - don't draw the debug view.
            if (Flags == 0)
                return;

            //_device.RasterizerState = RasterizerState.CullNone;
            //_device.DepthStencilState = DepthStencilState.Default;

			lineListBatch.Clear();
			triangleListBatch.Clear();
            //_primitiveBatch.Begin(ref projection, ref view);
            DrawDebugData();
            //_primitiveBatch.End();



            if ((Flags & DebugViewFlags.PerformanceGraph) == DebugViewFlags.PerformanceGraph)
            {
                //_primitiveBatch.Begin(ref _localProjection, ref _localView);
                DrawPerformanceGraph();
                //_primitiveBatch.End();
            }

			GL.PushMatrix();
			mat.SetPass(0);
			//GL.LoadOrtho();
			//GL.MultMatrix(projection);

			GL.Begin(GL.LINES);
			int nlines = lineListBatch.Count;
			for(int i = 0; i < nlines; i++)
			{
				GL.Color(lineListBatch[i].Color);
				GL.Vertex(lineListBatch[i].Position[0]);
				GL.Vertex(lineListBatch[i].Position[1]);
			}
			GL.End();
			GL.Begin(GL.TRIANGLES);
			nlines = triangleListBatch.Count;
			for(int i = 0; i < nlines; i++)
			{
				GL.Color(triangleListBatch[i].Color);
				GL.Vertex(triangleListBatch[i].Position[0]);
				GL.Vertex(triangleListBatch[i].Position[1]);
				GL.Vertex(triangleListBatch[i].Position[2]);
			}
			GL.End();

			GL.PopMatrix();

            // begin the sprite batch effect
            //_batch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);

            // draw any strings we have
            //for (int i = 0; i < _stringData.Count; i++)
            //{
            //    _batch.DrawString(_font, string.Format(_stringData[i].S, _stringData[i].Args),
            //                      new TSVector2(_stringData[i].X + 1f, _stringData[i].Y + 1f), Color.Black);
            //    _batch.DrawString(_font, string.Format(_stringData[i].S, _stringData[i].Args),
            //                      new TSVector2(_stringData[i].X, _stringData[i].Y), _stringData[i].Color);
            //}
            // end the sprite batch effect
            //_batch.End();

            //_stringData.Clear();

        }

        public void RenderDebugData(ref UnityEngine.Matrix4x4 projection, UnityEngine.Material mat)
        {
            if (!Enabled)
            {
                return;
            }
            UnityEngine.Matrix4x4 view = UnityEngine.Matrix4x4.identity;
            RenderDebugData(ref projection, ref view, mat);
        }

        //public void LoadContent(GraphicsDevice device, ContentManager content)
        //{
            // Create a new SpriteBatch, which can be used to draw textures.
            //_device = device;
            //_batch = new SpriteBatch(_device);
            //_primitiveBatch = new PrimitiveBatch(_device, 1000);
            //_font = content.Load<SpriteFont>("font");
            //_stringData = new List<StringData>();

        //    _localProjection = TSMatrix4x4.CreateOrthographicOffCenter(0f, _device.Viewport.Width, _device.Viewport.Height,
        //                                                          0f, 0f, 1f);
        //    _localView = TSMatrix4x4.Identity;
        //}

        #region Nested type: ContactPoint

        private struct ContactPoint
        {
            public TSVector2 Normal;
            public TSVector2 Position;
            public PointState State;
        }

        #endregion

        #region Nested type: StringData

        private struct StringData
        {
            public object[] Args;
            public Color Color;
            public string S;
            public int X, Y;

            public StringData(int x, int y, string s, object[] args, Color color)
            {
                X = x;
                Y = y;
                S = s;
                Args = args;
                Color = color;
            }
        }

        #endregion
    }
	public struct FDVertex
	{
		public Color Color;
		public Vector3[] Position;

		public static FDVertex FromLine(TSVector begin, TSVector end, Color color)
		{
			FDVertex fv = new FDVertex();
			fv.Position = new Vector3[2];
			fv.Position[0] = FSHelper.TSVector3ToVector3(begin);
			fv.Position[1] = FSHelper.TSVector3ToVector3(end);
			fv.Color = color;
			return fv;
		}
		public static FDVertex FromTriangleList(TSVector p0, TSVector p1, TSVector p2, Color color)
		{
			FDVertex fv = new FDVertex();
			fv.Position = new Vector3[3];
			fv.Position[0] = FSHelper.TSVector3ToVector3(p0);
			fv.Position[1] = FSHelper.TSVector3ToVector3(p1);
			fv.Position[2] = FSHelper.TSVector3ToVector3(p2);
			fv.Color = color;
			return fv;
		}
		public static FDVertex FromLine(TSVector2 begin, TSVector2 end, Color color)
		{
			FDVertex fv = new FDVertex();
			fv.Position = new Vector3[2];
			fv.Position[0] = FSHelper.TSVector2ToVector3(begin);
			fv.Position[1] = FSHelper.TSVector2ToVector3(end);
			fv.Color = color;
			return fv;
		}
		public static FDVertex FromTriangleList(TSVector2 p0, TSVector2 p1, TSVector2 p2, Color color)
		{
			FDVertex fv = new FDVertex();
			fv.Position = new Vector3[3];
			fv.Position[0] = FSHelper.TSVector2ToVector3(p0);
			fv.Position[1] = FSHelper.TSVector2ToVector3(p1);
			fv.Position[2] = FSHelper.TSVector2ToVector3(p2);
			fv.Color = color;
			return fv;
		}
	}
}