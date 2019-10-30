using System.Diagnostics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints
{
    /// <summary>
    /// Maintains a fixed angle between two bodies
    /// </summary>
    public class AngleJoint : FarseerJoint
    {
        public FixedPoint BiasFactor;
        public FixedPoint MaxImpulse;
        public FixedPoint Softness;
        private FixedPoint _bias;
        private FixedPoint _jointError;
        private FixedPoint _massFactor;
        private FixedPoint _targetAngle;

        internal AngleJoint() {
            JointType = JointType.Angle;
        }

        public AngleJoint(Body bodyA, Body bodyB)
            : base(bodyA, bodyB) {
            JointType = JointType.Angle;
            TargetAngle = 0;
            BiasFactor = .2f;
            Softness = 0f;
            MaxImpulse = FixedPoint.MaxValue;
        }

        public FixedPoint TargetAngle {
            get { return _targetAngle; }
            set {
                if (value != _targetAngle) {
                    _targetAngle = value;
                    WakeBodies();
                }
            }
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.Position; }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.Position; }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            //TODO
            //return _inv_dt * _impulse;
            return TSVector2.zero;
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            return 0;
        }

        internal override void InitVelocityConstraints(ref SolverData data) {
            _jointError = (BodyB.Sweep.A - BodyA.Sweep.A - TargetAngle);

            _bias = -BiasFactor * data.step.inv_dt * _jointError;

            _massFactor = (1 - Softness) / (BodyA.InvI + BodyB.InvI);
        }

        internal override void SolveVelocityConstraints(ref SolverData data) {
            //GABS: NOT A BOTTLENECK

            FixedPoint p = (_bias - BodyB.AngularVelocity + BodyA.AngularVelocity) * _massFactor;
            BodyA.AngularVelocity -= BodyA.InvI * TSMath.Sign(p) * TSMath.Min(TSMath.Abs(p), MaxImpulse);
            BodyB.AngularVelocity += BodyB.InvI * TSMath.Sign(p) * TSMath.Min(TSMath.Abs(p), MaxImpulse);
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            //no position solving for this joint
            return true;
        }
    }
}