using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// An easy to use factory for creating bodies
    /// </summary>
    public static class FixtureFactory
    {
        public static Fixture AttachEdge(TSVector2 start, TSVector2 end, Body body, object userData = null) {
            EdgeShape edgeShape = new EdgeShape(start, end);
            return body.CreateFixture(edgeShape, userData);
        }

        public static Fixture AttachChainShape(Vertices vertices, Body body, object userData = null) {
            ChainShape shape = new ChainShape(vertices);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachLoopShape(Vertices vertices, Body body, object userData = null) {
            ChainShape shape = new ChainShape(vertices, true);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachRectangle(FixedPoint width, FixedPoint height, TSVector2 offset, Body body,
            object userData = null) {
            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / 2, height / 2);
            rectangleVertices.Translate(ref offset);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices);
            return body.CreateFixture(rectangleShape, userData);
        }

        public static Fixture AttachCircle(FixedPoint radius, Body body, object userData = null) {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius);
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachCircle(FixedPoint radius, Body body, TSVector2 offset, object userData = null) {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius);
            circleShape.Position = offset;
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachPolygon(Vertices vertices, Body body, object userData = null) {
            if (vertices.Count <= 1)
                throw new ArgumentOutOfRangeException("vertices", "Too few points to be a polygon");

            PolygonShape polygon = new PolygonShape(vertices);
            return body.CreateFixture(polygon, userData);
        }

        public static Fixture AttachEllipse(FixedPoint xRadius, FixedPoint yRadius, int edges, Body body,
            object userData = null) {
            if (xRadius <= 0)
                throw new ArgumentOutOfRangeException("xRadius", "X-radius must be more than 0");

            if (yRadius <= 0)
                throw new ArgumentOutOfRangeException("yRadius", "Y-radius must be more than 0");

            Vertices ellipseVertices = PolygonTools.CreateEllipse(xRadius, yRadius, edges);
            PolygonShape polygonShape = new PolygonShape(ellipseVertices);
            return body.CreateFixture(polygonShape, userData);
        }

        public static List<Fixture> AttachCompoundPolygon(List<Vertices> list, Body body, object userData = null) {
            List<Fixture> res = new List<Fixture>(list.Count);

            //Then we create several fixtures using the body
            foreach (Vertices vertices in list) {
                if (vertices.Count == 2) {
                    EdgeShape shape = new EdgeShape(vertices[0], vertices[1]);
                    res.Add(body.CreateFixture(shape, userData));
                }
                else {
                    PolygonShape shape = new PolygonShape(vertices);
                    res.Add(body.CreateFixture(shape, userData));
                }
            }

            return res;
        }

        public static Fixture AttachLineArc(FixedPoint radians, int sides, FixedPoint radius, TSVector2 position,
            FixedPoint angle, bool closed, Body body) {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((FixedPoint.Pi - radians) / 2 + angle);
            arc.Translate(ref position);

            return closed ? AttachLoopShape(arc, body) : AttachChainShape(arc, body);
        }

        public static List<Fixture> AttachSolidArc(FixedPoint radians, int sides,
            FixedPoint radius, TSVector2 position, FixedPoint angle, Body body) {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((FixedPoint.Pi - radians) / 2 + angle);

            arc.Translate(ref position);

            //Close the arc
            arc.Add(arc[0]);

            List<Vertices> triangles =
                Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip, true, FixedPoint.EN3);

            return AttachCompoundPolygon(triangles, body);
        }
    }
}