using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints;

namespace vFrame.Lockstep.Core.FarseerPhysics.Factories
{
    /// <summary>
    /// An easy to use factory for using joints.
    /// </summary>
    public static class JointFactory
    {
        #region Revolute Joint

        /// <summary>
        /// Creates a revolute joint.
        /// </summary>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="localAnchorB">The anchor of bodyB in local coordinates</param>
        /// <returns></returns>
        public static RevoluteJoint CreateRevoluteJoint(Body bodyA, Body bodyB, TSVector2 localAnchorB) {
            TSVector2 localanchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(localAnchorB));
            RevoluteJoint joint = new RevoluteJoint(bodyA, bodyB, localanchorA, localAnchorB);
            return joint;
        }

        /// <summary>
        /// Creates a revolute joint and adds it to the world
        /// </summary>
        /// <param name="world"></param>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="anchor"></param>
        /// <returns></returns>
        public static RevoluteJoint CreateRevoluteJoint(World world, Body bodyA, Body bodyB, TSVector2 anchor) {
            RevoluteJoint joint = CreateRevoluteJoint(bodyA, bodyB, anchor);
            world.AddJoint(joint);
            return joint;
        }

        ///// <summary>
        ///// Creates the fixed revolute joint.
        ///// </summary>
        ///// <param name="world">The world.</param>
        ///// <param name="body">The body.</param>
        ///// <param name="bodyAnchor">The body anchor.</param>
        ///// <param name="worldAnchor">The world anchor.</param>
        ///// <returns></returns>
        //public static FixedRevoluteJoint CreateFixedRevoluteJoint(World world, Body body, Vector2 bodyAnchor,
        //                                                          Vector2 worldAnchor)
        //{
        //    FixedRevoluteJoint fixedRevoluteJoint = new FixedRevoluteJoint(body, bodyAnchor, worldAnchor);
        //    world.AddJoint(fixedRevoluteJoint);
        //    return fixedRevoluteJoint;
        //}

        #endregion

        #region Weld Joint

        /// <summary>
        /// Creates a weld joint
        /// </summary>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="worldAnchor">World space coordinates of weld joint</param>
        /// <returns></returns>
        public static WeldJoint CreateWeldJoint(Body bodyA, Body bodyB, TSVector2 worldAnchor) {
            WeldJoint joint = new WeldJoint(bodyA, bodyB, bodyA.GetLocalPoint(worldAnchor),
                bodyB.GetLocalPoint(worldAnchor));
            return joint;
        }

        /// <summary>
        /// Creates a weld joint and adds it to the world
        /// </summary>
        /// <param name="world"></param>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="worldAnchor">World space coordinates of weld joint</param>
        /// <returns></returns>
        public static WeldJoint CreateWeldJoint(World world, Body bodyA, Body bodyB, TSVector2 worldAnchor) {
            WeldJoint joint = CreateWeldJoint(bodyA, bodyB, worldAnchor);
            world.AddJoint(joint);
            return joint;
        }

        public static WeldJoint CreateWeldJoint(World world, Body bodyA, Body bodyB, TSVector2 localAnchorA,
            TSVector2 localAnchorB) {
            WeldJoint weldJoint = new WeldJoint(bodyA, bodyB, localAnchorA, localAnchorB);
            world.AddJoint(weldJoint);
            return weldJoint;
        }

        #endregion

        #region Prismatic Joint

        /// <summary>
        /// Creates a prsimatic joint
        /// </summary>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="localanchorB"></param>
        /// <param name="axis"></param>
        /// <returns></returns>
        public static PrismaticJoint
            CreatePrismaticJoint(Body bodyA, Body bodyB, TSVector2 localanchorB, TSVector2 axis) {
            TSVector2 localanchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(localanchorB));
            PrismaticJoint joint = new PrismaticJoint(bodyA, bodyB, localanchorA, localanchorB, axis);
            return joint;
        }

        /// <summary>
        /// Creates a prismatic joint and adds it to the world
        /// </summary>
        /// <param name="world"></param>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="localanchorB"></param>
        /// <param name="axis"></param>
        /// <returns></returns>
        public static PrismaticJoint CreatePrismaticJoint(World world, Body bodyA, Body bodyB, TSVector2 localanchorB,
            TSVector2 axis) {
            PrismaticJoint joint = CreatePrismaticJoint(bodyA, bodyB, localanchorB, axis);
            world.AddJoint(joint);
            return joint;
        }

        //public static FixedPrismaticJoint CreateFixedPrismaticJoint(World world, Body body, Vector2 worldAnchor,
        //                                                            Vector2 axis)
        //{
        //    FixedPrismaticJoint joint = new FixedPrismaticJoint(body, worldAnchor, axis);
        //    world.AddJoint(joint);
        //    return joint;
        //}

        #endregion

        #region Wheel Joint

        /// <summary>
        /// Creates a Wheel Joint
        /// </summary>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="anchor"></param>
        /// <param name="axis"></param>
        /// <returns></returns>
        public static WheelJoint CreateWheelJoint(Body bodyA, Body bodyB, TSVector2 anchor, TSVector2 axis) {
            WheelJoint joint = new WheelJoint(bodyA, bodyB, anchor, axis);
            return joint;
        }

        /// <summary>
        /// Creates a Wheel Joint and adds it to the world
        /// </summary>
        /// <param name="world"></param>
        /// <param name="bodyA"></param>
        /// <param name="bodyB"></param>
        /// <param name="localanchorB"></param>
        /// <param name="axis"></param>
        /// <returns></returns>
        public static WheelJoint CreateWheelJoint(World world, Body bodyA, Body bodyB, TSVector2 localanchorB,
            TSVector2 axis) {
            WheelJoint joint = CreateWheelJoint(bodyA, bodyB, localanchorB, axis);
            world.AddJoint(joint);
            return joint;
        }

        #endregion

        #region Angle Joint

        /// <summary>
        /// Creates an angle joint.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <returns></returns>
        public static AngleJoint CreateAngleJoint(World world, Body bodyA, Body bodyB) {
            AngleJoint angleJoint = new AngleJoint(bodyA, bodyB);
            world.AddJoint(angleJoint);

            return angleJoint;
        }

        /// <summary>
        /// Creates a fixed angle joint.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="body">The body.</param>
        /// <returns></returns>
        public static FixedAngleJoint CreateFixedAngleJoint(World world, Body body) {
            FixedAngleJoint angleJoint = new FixedAngleJoint(body);
            world.AddJoint(angleJoint);

            return angleJoint;
        }

        #endregion

        #region Distance Joint

        public static DistanceJoint CreateDistanceJoint(World world, Body bodyA, Body bodyB, TSVector2 anchorA,
            TSVector2 anchorB) {
            DistanceJoint distanceJoint = new DistanceJoint(bodyA, bodyB, anchorA, anchorB);
            world.AddJoint(distanceJoint);
            return distanceJoint;
        }

        //public static FixedDistanceJoint CreateFixedDistanceJoint(World world, Body body, Vector2 localAnchor,
        //                                                          Vector2 worldAnchor)
        //{
        //    FixedDistanceJoint distanceJoint = new FixedDistanceJoint(body, localAnchor, worldAnchor);
        //    world.AddJoint(distanceJoint);
        //    return distanceJoint;
        //}

        #endregion

        #region Friction Joint

        public static FrictionJoint CreateFrictionJoint(World world, Body bodyA, Body bodyB, TSVector2 anchorA,
            TSVector2 anchorB) {
            FrictionJoint frictionJoint = new FrictionJoint(bodyA, bodyB, anchorA, anchorB);
            world.AddJoint(frictionJoint);
            return frictionJoint;
        }

        //public static FixedFrictionJoint CreateFixedFrictionJoint(World world, Body body, Vector2 bodyAnchor)
        //{
        //    FixedFrictionJoint frictionJoint = new FixedFrictionJoint(body, bodyAnchor);
        //    world.AddJoint(frictionJoint);
        //    return frictionJoint;
        //}

        #endregion

        #region Gear Joint

        public static GearJoint CreateGearJoint(World world, FarseerJoint jointA, FarseerJoint jointB, FixedPoint ratio) {
            GearJoint gearJoint = new GearJoint(jointA, jointB, ratio);
            world.AddJoint(gearJoint);
            return gearJoint;
        }

        #endregion

        #region Pulley Joint

        public static PulleyJoint CreatePulleyJoint(World world, Body bodyA, Body bodyB, TSVector2 groundAnchorA,
            TSVector2 groundAnchorB, TSVector2 anchorA, TSVector2 anchorB, FixedPoint ratio) {
            PulleyJoint pulleyJoint = new PulleyJoint(bodyA, bodyB, groundAnchorA, groundAnchorB, anchorA, anchorB,
                ratio);
            world.AddJoint(pulleyJoint);
            return pulleyJoint;
        }

        #endregion

        #region Slider Joint

        public static SliderJoint CreateSliderJoint(World world, Body bodyA, Body bodyB, TSVector2 anchorA,
            TSVector2 anchorB, FixedPoint minLength, FixedPoint maxLength) {
            SliderJoint sliderJoint = new SliderJoint(bodyA, bodyB, anchorA, anchorB, minLength, maxLength);
            world.AddJoint(sliderJoint);
            return sliderJoint;
        }

        #endregion

        public static FixedMouseJoint CreateFixedMouseJoint(World world, Body body, TSVector2 target) {
            FixedMouseJoint joint = new FixedMouseJoint(body, target);
            world.AddJoint(joint);
            return joint;
        }
    }
}