using System.Diagnostics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints
{
    public class FixedAngleJoint : FarseerJoint
    {
        public FixedPoint BiasFactor;
        public FixedPoint MaxImpulse;
        public FixedPoint Softness;
        private FixedPoint _bias;
        private FixedPoint _jointError;
        private FixedPoint _massFactor;
        private FixedPoint _targetAngle;

        public FixedAngleJoint(Body bodyA)
            : base(bodyA) {
            JointType = JointType.FixedAngle;
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
            get { return BodyA.Position; }
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
            _jointError = BodyA.Sweep.A - TargetAngle;

            _bias = -BiasFactor * data.step.inv_dt * _jointError;

            _massFactor = (1 - Softness) / (BodyA.InvI);
        }

        internal override void SolveVelocityConstraints(ref SolverData data) {
            FixedPoint p = (_bias - BodyA.AngularVelocity) * _massFactor;
            BodyA.AngularVelocity += BodyA.InvI * TSMath.Sign(p) * TSMath.Min(TSMath.Abs(p), MaxImpulse);
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            //no position solving for this joint
            return true;
        }
    }
}