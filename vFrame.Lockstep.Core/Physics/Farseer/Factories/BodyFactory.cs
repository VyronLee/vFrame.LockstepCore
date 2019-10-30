using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.Physics2D
{
    public static class BodyFactory
    {
        public static Body CreateBody(World world, object userData = null) {
            Body body = new Body(world, null, TSVector2.right, userData);
            return body;
        }

        // TS - public static Body CreateBody(World world, Vector2 position, FP rotation = 0, object userData = null)
        public static Body CreateBody(World world, TSVector2 position, TSVector2 forward, object userData = null) {
            Body body = new Body(world, position, forward, userData);
            return body;
        }

        public static Body CreateEdge(World world, TSVector2 start, TSVector2 end, object userData = null) {
            Body body = CreateBody(world);
            FixtureFactory.AttachEdge(start, end, body, userData);
            return body;
        }

        public static Body CreateChainShape(World world, Vertices vertices, object userData = null) {
            return CreateChainShape(world, vertices, TSVector2.zero, userData);
        }

        public static Body CreateChainShape(World world, Vertices vertices, TSVector2 position,
            object userData = null) {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachChainShape(vertices, body, userData);
            return body;
        }

        public static Body CreateLoopShape(World world, Vertices vertices, object userData = null) {
            return CreateLoopShape(world, vertices, TSVector2.zero, userData);
        }

        public static Body CreateLoopShape(World world, Vertices vertices, TSVector2 position, object userData = null) {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachLoopShape(vertices, body, userData);
            return body;
        }

        public static Body CreateRectangle(World world, FixedPoint width, FixedPoint height, object userData = null) {
            return CreateRectangle(world, width, height, TSVector2.zero, userData);
        }

        public static Body CreateRectangle(World world, FixedPoint width, FixedPoint height, TSVector2 position,
            object userData = null) {
            if (width <= 0)
                throw new ArgumentOutOfRangeException("width", "Width must be more than 0 meters");

            if (height <= 0)
                throw new ArgumentOutOfRangeException("height", "Height must be more than 0 meters");

            Body newBody = CreateBody(world, position);
            newBody.UserData = userData;

            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / 2, height / 2);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices);
            newBody.CreateFixture(rectangleShape);

            return newBody;
        }

        public static Body CreateCircle(World world, FixedPoint radius, object userData = null) {
            return CreateCircle(world, radius, TSVector2.zero, userData);
        }

        public static Body CreateCircle(World world, FixedPoint radius, TSVector2 position, object userData = null) {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachCircle(radius, body, userData);
            return body;
        }

        public static Body CreateEllipse(World world, FixedPoint xRadius, FixedPoint yRadius, int edges,
            TSVector2 position, object userData = null) {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachEllipse(xRadius, yRadius, edges, body, userData);
            return body;
        }

        public static Body CreatePolygon(World world, Vertices vertices, object userData) {
            return CreatePolygon(world, vertices, TSVector2.zero, userData);
        }

        public static Body CreatePolygon(World world, Vertices vertices, TSVector2 position, object userData) {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachPolygon(vertices, body, userData);
            return body;
        }
    }
}