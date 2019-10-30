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

using System.Diagnostics;
using vFrame.Lockstep.Core.FarseerPhysics.Common;

namespace vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints
{
    // Linear constraint (point-to-line)
    // d = p2 - p1 = x2 + r2 - x1 - r1
    // C = dot(perp, d)
    // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    //
    // Angular constraint
    // C = a2 - a1 + a_initial
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    //
    // K = J * invM * JT
    //
    // J = [-a -s1 a s2]
    //     [0  -1  0  1]
    // a = perp
    // s1 = cross(d + r1, a) = cross(p2 - x1, a)
    // s2 = cross(r2, a) = cross(p2 - x2, a)
    // Motor/Limit linear constraint
    // C = dot(ax1, d)
    // Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    // Block Solver
    // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    // when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    // The Jacobian has 3 rows:
    // J = [-uT -s1 uT s2] // linear
    //     [0   -1   0  1] // angular
    //     [-vT -a1 vT a2] // limit
    //
    // u = perp
    // v = axis
    // s1 = cross(d + r1, u), s2 = cross(r2, u)
    // a1 = cross(d + r1, v), a2 = cross(r2, v)
    // M * (v2 - v1) = JT * df
    // J * v2 = bias
    //
    // v2 = v1 + invM * JT * df
    // J * (v1 + invM * JT * df) = bias
    // K * df = bias - J * v1 = -Cdot
    // K = J * invM * JT
    // Cdot = J * v1 - bias
    //
    // Now solve for f2.
    // df = f2 - f1
    // K * (f2 - f1) = -Cdot
    // f2 = invK * (-Cdot) + f1
    //
    // Clamp accumulated limit impulse.
    // lower: f2(3) = max(f2(3), 0)
    // upper: f2(3) = min(f2(3), 0)
    //
    // Solve for correct f2(1:2)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    // Now compute impulse to be applied:
    // df = f2 - f1

    /// <summary>
    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in body1. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    /// </summary>
    public class PrismaticJoint : FarseerJoint
    {
        public TSVector2 LocalAnchorA;
        public TSVector2 LocalAnchorB;
        private TSVector2 _localXAxisA;
        private TSVector2 _localYAxisA;
        private FixedPoint m_referenceAngle;
        private TSVector _impulse;
        private FixedPoint _motorImpulse;
        private FixedPoint _lowerTranslation;
        private FixedPoint _upperTranslation;
        private FixedPoint _maxMotorForce;
        private FixedPoint _motorSpeed;
        private bool _enableLimit;
        private bool _enableMotor;
        private LimitState _limitState;

        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private TSVector2 m_localCenterA;
        private TSVector2 m_localCenterB;
        private FixedPoint m_invMassA;
        private FixedPoint m_invMassB;
        private FixedPoint m_invIA;
        private FixedPoint m_invIB;
        private TSVector2 m_axis, m_perp;
        private FixedPoint m_s1, m_s2;
        private FixedPoint m_a1, m_a2;
        private Mat33 m_K;
        private FixedPoint m_motorMass;

        internal PrismaticJoint() {
            JointType = JointType.Prismatic;
        }

        /// <summary>
        /// This requires defining a line of
        /// motion using an axis and an anchor point. The definition uses local
        /// anchor points and a local axis so that the initial configuration
        /// can violate the constraint slightly. The joint translation is zero
        /// when the local anchor points coincide in world space. Using local
        /// anchors and a local axis helps when saving and loading a game.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="localAnchorA">The first body anchor.</param>
        /// <param name="localAnchorB">The second body anchor.</param>
        /// <param name="axis">The axis.</param>
        public PrismaticJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB, TSVector2 axis)
            : base(bodyA, bodyB) {
            JointType = JointType.Prismatic;

            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;

            _localXAxisA = BodyA.GetLocalVector(axis);
            _localXAxisA.Normalize();
            _localYAxisA = MathUtils.Cross(1.0f, _localXAxisA);
            m_referenceAngle = BodyB.Rotation - BodyA.Rotation;

            _limitState = LimitState.Inactive;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// Get the current joint translation, usually in meters.
        /// </summary>
        /// <value></value>
        public FixedPoint JointTranslation {
            get {
                TSVector2 d = BodyB.GetWorldPoint(LocalAnchorB) - BodyA.GetWorldPoint(LocalAnchorA);
                TSVector2 axis = BodyA.GetWorldVector(ref _localXAxisA);

                return TSVector2.Dot(d, axis);
            }
        }

        /// <summary>
        /// Get the current joint translation speed, usually in meters per second.
        /// </summary>
        /// <value></value>
        public FixedPoint JointSpeed {
            get {
                Transform xf1, xf2;
                BodyA.GetTransform(out xf1);
                BodyB.GetTransform(out xf2);

                TSVector2 r1 = MathUtils.Mul(ref xf1.q, LocalAnchorA - BodyA.LocalCenter);
                TSVector2 r2 = MathUtils.Mul(ref xf2.q, LocalAnchorB - BodyB.LocalCenter);
                TSVector2 p1 = BodyA.Sweep.C + r1;
                TSVector2 p2 = BodyB.Sweep.C + r2;
                TSVector2 d = p2 - p1;
                TSVector2 axis = BodyA.GetWorldVector(ref _localXAxisA);

                TSVector2 v1 = BodyA.LinearVelocityInternal;
                TSVector2 v2 = BodyB.LinearVelocityInternal;
                FixedPoint w1 = BodyA.AngularVelocityInternal;
                FixedPoint w2 = BodyB.AngularVelocityInternal;

                FixedPoint speed = TSVector2.Dot(d, MathUtils.Cross(w1, axis)) +
                              TSVector2.Dot(axis, v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1));
                return speed;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
        public bool LimitEnabled {
            get { return _enableLimit; }
            set {
                Debug.Assert(BodyA.FixedRotation == false || BodyB.FixedRotation == false,
                    "Warning: limits does currently not work with fixed rotation");

                if (value != _enableLimit) {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.z = 0;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public FixedPoint LowerLimit {
            get { return _lowerTranslation; }
            set {
                if (value != _lowerTranslation) {
                    WakeBodies();
                    _lowerTranslation = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public FixedPoint UpperLimit {
            get { return _upperTranslation; }
            set {
                if (value != _upperTranslation) {
                    WakeBodies();
                    _upperTranslation = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower"></param>
        /// <param name="upper"></param>
        public void SetLimits(FixedPoint lower, FixedPoint upper) {
            if (upper != _upperTranslation || lower != _lowerTranslation) {
                WakeBodies();
                _upperTranslation = upper;
                _lowerTranslation = lower;
                _impulse.z = 0.0f;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <value><c>true</c> if [motor enabled]; otherwise, <c>false</c>.</value>
        public bool MotorEnabled {
            get { return _enableMotor; }
            set {
                WakeBodies();
                _enableMotor = value;
            }
        }

        /// <summary>
        /// Set the motor speed, usually in meters per second.
        /// </summary>
        /// <value>The speed.</value>
        public FixedPoint MotorSpeed {
            set {
                WakeBodies();
                _motorSpeed = value;
            }
            get { return _motorSpeed; }
        }

        /// <summary>
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <value>The force.</value>
        public FixedPoint MaxMotorForce {
            get { return _maxMotorForce; }
            set {
                WakeBodies();
                _maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor impulse, usually in N.
        /// </summary>
        /// <value></value>
        public FixedPoint MotorImpulse {
            get { return _motorImpulse; }
            set { _motorImpulse = value; }
        }

        public FixedPoint GetMotorForce(FixedPoint inv_dt) {
            return inv_dt * _motorImpulse;
        }

        public TSVector2 LocalXAxisA {
            get { return _localXAxisA; }
            set {
                _localXAxisA = BodyA.GetLocalVector(value);
                _localYAxisA = MathUtils.Cross(1.0f, _localXAxisA);
            }
        }

        public FixedPoint ReferenceAngle {
            get { return m_referenceAngle; }
            set { m_referenceAngle = value; }
        }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            return inv_dt * (_impulse.x * m_perp + (_motorImpulse + _impulse.z) * m_axis);
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            return inv_dt * _impulse.y;
        }

        internal override void InitVelocityConstraints(ref SolverData data) {
            m_indexA = BodyA.IslandIndex;
            m_indexB = BodyB.IslandIndex;
            m_localCenterA = BodyA.Sweep.LocalCenter;
            m_localCenterB = BodyB.Sweep.LocalCenter;
            m_invMassA = BodyA.InvMass;
            m_invMassB = BodyB.InvMass;
            m_invIA = BodyA.InvI;
            m_invIB = BodyB.InvI;

            TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;

            TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            // Compute the effective masses.
            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);
            TSVector2 d = (cB - cA) + rB - rA;

            FixedPoint mA = m_invMassA, mB = m_invMassB;
            FixedPoint iA = m_invIA, iB = m_invIB;

            // Compute motor Jacobian and effective mass.
            {
                m_axis = MathUtils.Mul(qA, LocalXAxisA);
                m_a1 = MathUtils.Cross(d + rA, m_axis);
                m_a2 = MathUtils.Cross(rB, m_axis);

                m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
                if (m_motorMass > 0.0f) {
                    m_motorMass = 1.0f / m_motorMass;
                }
            }

            // Prismatic constraint.
            {
                m_perp = MathUtils.Mul(qA, _localYAxisA);

                m_s1 = MathUtils.Cross(d + rA, m_perp);
                m_s2 = MathUtils.Cross(rB, m_perp);

                FixedPoint k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
                FixedPoint k12 = iA * m_s1 + iB * m_s2;
                FixedPoint k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
                FixedPoint k22 = iA + iB;
                if (k22 == 0.0f) {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }

                FixedPoint k23 = iA * m_a1 + iB * m_a2;
                FixedPoint k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

                m_K.ex = new TSVector(k11, k12, k13);
                m_K.ey = new TSVector(k12, k22, k23);
                m_K.ez = new TSVector(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (_enableLimit) {
                FixedPoint jointTranslation = TSVector2.Dot(m_axis, d);
                if (TSMath.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop) {
                    _limitState = LimitState.Equal;
                }
                else if (jointTranslation <= _lowerTranslation) {
                    if (_limitState != LimitState.AtLower) {
                        _limitState = LimitState.AtLower;
                        _impulse.z = 0.0f;
                    }
                }
                else if (jointTranslation >= _upperTranslation) {
                    if (_limitState != LimitState.AtUpper) {
                        _limitState = LimitState.AtUpper;
                        _impulse.z = 0.0f;
                    }
                }
                else {
                    _limitState = LimitState.Inactive;
                    _impulse.z = 0.0f;
                }
            }
            else {
                _limitState = LimitState.Inactive;
            }

            if (_enableMotor == false) {
                _motorImpulse = 0.0f;
            }

            if (Settings.EnableWarmstarting) {
                // Account for variable time step.
                _impulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                TSVector2 P = _impulse.x * m_perp + (_motorImpulse + _impulse.z) * m_axis;
                FixedPoint LA = _impulse.x * m_s1 + _impulse.y + (_motorImpulse + _impulse.z) * m_a1;
                FixedPoint LB = _impulse.x * m_s2 + _impulse.y + (_motorImpulse + _impulse.z) * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else {
                _impulse = TSVector.zero;
                _motorImpulse = 0.0f;
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data) {
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            FixedPoint mA = m_invMassA, mB = m_invMassB;
            FixedPoint iA = m_invIA, iB = m_invIB;

            // Solve linear motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal) {
                FixedPoint Cdot = TSVector2.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                FixedPoint impulse = m_motorMass * (_motorSpeed - Cdot);
                FixedPoint oldImpulse = _motorImpulse;
                FixedPoint maxImpulse = data.step.dt * _maxMotorForce;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                TSVector2 P = impulse * m_axis;
                FixedPoint LA = impulse * m_a1;
                FixedPoint LB = impulse * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            TSVector2 Cdot1 = new TSVector2();
            Cdot1.x = TSVector2.Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
            Cdot1.y = wB - wA;

            if (_enableLimit && _limitState != LimitState.Inactive) {
                // Solve prismatic and limit constraint in block form.
                FixedPoint Cdot2;
                Cdot2 = TSVector2.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector f1 = _impulse;
                TSVector df = m_K.Solve33(-1 * Cdot);
                _impulse += df;

                if (_limitState == LimitState.AtLower) {
                    _impulse.z = TSMath.Max(_impulse.z, 0.0f);
                }
                else if (_limitState == LimitState.AtUpper) {
                    _impulse.z = TSMath.Min(_impulse.z, 0.0f);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
                TSVector2 b = -Cdot1 - (_impulse.z - f1.z) * new TSVector2(m_K.ez.x, m_K.ez.y);
                TSVector2 f2r = m_K.Solve22(b) + new TSVector2(f1.x, f1.y);
                _impulse.x = f2r.x;
                _impulse.y = f2r.y;

                df = _impulse - f1;

                TSVector2 P = df.x * m_perp + df.z * m_axis;
                FixedPoint LA = df.x * m_s1 + df.y + df.z * m_a1;
                FixedPoint LB = df.x * m_s2 + df.y + df.z * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else {
                // Limit is inactive, just solve the prismatic constraint in block form.
                TSVector2 df = m_K.Solve22(-Cdot1);
                _impulse.x += df.x;
                _impulse.y += df.y;

                TSVector2 P = df.x * m_perp;
                FixedPoint LA = df.x * m_s1 + df.y;
                FixedPoint LB = df.x * m_s2 + df.y;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;

                //TSVector2 Cdot10 = Cdot1;

                Cdot1.x = TSVector2.Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
                Cdot1.y = wB - wA;

                if (TSMath.Abs(Cdot1.x) > 0.01f || TSMath.Abs(Cdot1.y) > 0.01f) {
                    //TSVector2 test = MathUtils.Mul22(m_K, df);
                    Cdot1.x += 0.0f;
                }
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            FixedPoint mA = m_invMassA, mB = m_invMassB;
            FixedPoint iA = m_invIA, iB = m_invIB;

            // Compute fresh Jacobians
            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);
            TSVector2 d = cB + rB - cA - rA;

            TSVector2 axis = MathUtils.Mul(qA, LocalXAxisA);
            FixedPoint a1 = MathUtils.Cross(d + rA, axis);
            FixedPoint a2 = MathUtils.Cross(rB, axis);
            TSVector2 perp = MathUtils.Mul(qA, _localYAxisA);

            FixedPoint s1 = MathUtils.Cross(d + rA, perp);
            FixedPoint s2 = MathUtils.Cross(rB, perp);

            TSVector impulse;
            TSVector2 C1 = new TSVector2();
            C1.x = TSVector2.Dot(perp, d);
            C1.y = aB - aA - m_referenceAngle;

            FixedPoint linearError = TSMath.Abs(C1.x);
            FixedPoint angularError = TSMath.Abs(C1.y);

            bool active = false;
            FixedPoint C2 = 0.0f;
            if (_enableLimit) {
                FixedPoint translation = TSVector2.Dot(axis, d);
                if (TSMath.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop) {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
                    linearError = TSMath.Max(linearError, TSMath.Abs(translation));
                    active = true;
                }
                else if (translation <= _lowerTranslation) {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.LinearSlop,
                        -Settings.MaxLinearCorrection, 0.0f);
                    linearError = TSMath.Max(linearError, _lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= _upperTranslation) {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.LinearSlop, 0.0f,
                        Settings.MaxLinearCorrection);
                    linearError = TSMath.Max(linearError, translation - _upperTranslation);
                    active = true;
                }
            }

            if (active) {
                FixedPoint k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                FixedPoint k12 = iA * s1 + iB * s2;
                FixedPoint k13 = iA * s1 * a1 + iB * s2 * a2;
                FixedPoint k22 = iA + iB;
                if (k22 == 0.0f) {
                    // For fixed rotation
                    k22 = 1.0f;
                }

                FixedPoint k23 = iA * a1 + iB * a2;
                FixedPoint k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                Mat33 K = new Mat33();
                K.ex = new TSVector(k11, k12, k13);
                K.ey = new TSVector(k12, k22, k23);
                K.ez = new TSVector(k13, k23, k33);

                TSVector C = new TSVector();
                C.x = C1.x;
                C.y = C1.y;
                C.z = C2;

                impulse = K.Solve33(-1 * C);
            }
            else {
                FixedPoint k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                FixedPoint k12 = iA * s1 + iB * s2;
                FixedPoint k22 = iA + iB;
                if (k22 == 0.0f) {
                    k22 = 1.0f;
                }

                Mat22 K = new Mat22();
                K.ex = new TSVector2(k11, k12);
                K.ey = new TSVector2(k12, k22);

                TSVector2 impulse1 = K.Solve(-C1);
                impulse = new TSVector();
                impulse.x = impulse1.x;
                impulse.y = impulse1.y;
                impulse.z = 0.0f;
            }

            TSVector2 P = impulse.x * perp + impulse.z * axis;
            FixedPoint LA = impulse.x * s1 + impulse.y + impulse.z * a1;
            FixedPoint LB = impulse.x * s2 + impulse.y + impulse.z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;
            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}