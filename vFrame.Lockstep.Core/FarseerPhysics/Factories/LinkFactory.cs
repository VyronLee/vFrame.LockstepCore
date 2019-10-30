using System.Collections.Generic;
using vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes;
using vFrame.Lockstep.Core.FarseerPhysics.Common;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Factories
{
    public static class LinkFactory
    {
        /// <summary>
        /// Creates a chain.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="start">The start.</param>
        /// <param name="end">The end.</param>
        /// <param name="linkWidth">The width.</param>
        /// <param name="linkHeight">The height.</param>
        /// <param name="fixStart">if set to <c>true</c> [fix start].</param>
        /// <param name="fixEnd">if set to <c>true</c> [fix end].</param>
        /// <param name="numberOfLinks">The number of links.</param>
        /// <param name="linkDensity">The link density.</param>
        /// <returns></returns>
        public static Path CreateChain(World world, TSVector2 start, TSVector2 end, FixedPoint linkWidth, FixedPoint linkHeight,
            bool fixStart, bool fixEnd, int numberOfLinks, FixedPoint linkDensity) {
            //Chain start / end
            Path path = new Path();
            path.Add(start);
            path.Add(end);

            //A single chainlink
            PolygonShape shape = new PolygonShape(PolygonTools.CreateRectangle(linkWidth, linkHeight), linkDensity);

            //Use PathManager to create all the chainlinks based on the chainlink created before.
            List<Body> chainLinks =
                PathManager.EvenlyDistributeShapesAlongPath(world, path, shape, BodyType.Dynamic, numberOfLinks);

            //if (fixStart)
            //{
            //    //Fix the first chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(world, chainLinks[0], new Vector2(0, -(linkHeight / 2)),
            //                                          chainLinks[0].Position);
            //}

            //if (fixEnd)
            //{
            //    //Fix the last chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(world, chainLinks[chainLinks.Count - 1],
            //                                          new Vector2(0, (linkHeight / 2)),
            //                                          chainLinks[chainLinks.Count - 1].Position);
            //}

            //Attach all the chainlinks together with a revolute joint
            PathManager.AttachBodiesWithRevoluteJoint(world, chainLinks, new TSVector2(0, -linkHeight),
                new TSVector2(0, linkHeight),
                false, false);

            return (path);
        }
    }
}