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
    /// <summary>
    /// A revolute joint rains to bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    /// </summary>
    public class RevoluteJoint : FarseerJoint
    {
        // Solver shared
        public TSVector2 LocalAnchorA;
        public TSVector2 LocalAnchorB;

        private TSVector _impulse;
        private FixedPoint _motorImpulse;

        private bool _enableMotor;
        private FixedPoint _maxMotorTorque;
        private FixedPoint _motorSpeed;

        private bool _enableLimit;
        private FixedPoint _referenceAngle;
        private FixedPoint _lowerAngle;
        private FixedPoint _upperAngle;

        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private TSVector2 m_rA;
        private TSVector2 m_rB;
        private TSVector2 m_localCenterA;
        private TSVector2 m_localCenterB;
        private FixedPoint m_invMassA;
        private FixedPoint m_invMassB;
        private FixedPoint m_invIA;
        private FixedPoint m_invIB;
        private Mat33 m_mass; // effective mass for point-to-point constraint.
        private FixedPoint m_motorMass; // effective mass for motor/limit angular constraint.
        private LimitState _limitState;

        internal RevoluteJoint() {
            JointType = JointType.Revolute;
        }

        /// <summary>
        /// Initialize the bodies and local anchor.
        /// This requires defining an
        /// anchor point where the bodies are joined. The definition
        /// uses local anchor points so that the initial configuration
        /// can violate the constraint slightly. You also need to
        /// specify the initial relative angle for joint limits. This
        /// helps when saving and loading a game.
        /// The local anchor points are measured from the body's origin
        /// rather than the center of mass because:
        /// 1. you might not know where the center of mass will be.
        /// 2. if you add/remove shapes from a body and recompute the mass,
        /// the joints will be broken.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="localAnchorA">The first body anchor.</param>
        /// <param name="localAnchorB">The second anchor.</param>
        public RevoluteJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB)
            : base(bodyA, bodyB) {
            JointType = JointType.Revolute;

            // Changed to local coordinates.
            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;

            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _impulse = TSVector.zero;
            _limitState = LimitState.Inactive;
        }

        /// <summary>
        /// Initialize the bodies and world anchor.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="worldAnchor">The world coordinate anchor.</param>
        public RevoluteJoint(Body bodyA, Body bodyB, TSVector2 worldAnchor)
            : base(bodyA, bodyB) {
            JointType = JointType.Revolute;

            // Changed to local coordinates.
            LocalAnchorA = bodyA.GetLocalPoint(worldAnchor);
            LocalAnchorB = bodyB.GetLocalPoint(worldAnchor);

            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _impulse = TSVector.zero;

            _limitState = LimitState.Inactive;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public FixedPoint ReferenceAngle {
            get { return _referenceAngle; }
            set {
                WakeBodies();
                _referenceAngle = value;
            }
        }

        /// <summary>
        /// Get the current joint angle in radians.
        /// </summary>
        /// <value></value>
        public FixedPoint JointAngle {
            get { return BodyB.Sweep.A - BodyA.Sweep.A - ReferenceAngle; }
        }

        /// <summary>
        /// Get the current joint angle speed in radians per second.
        /// </summary>
        /// <value></value>
        public FixedPoint JointSpeed {
            get { return BodyB.AngularVelocityInternal - BodyA.AngularVelocityInternal; }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
        public bool LimitEnabled {
            get { return _enableLimit; }
            set {
                if (_enableLimit != value) {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit in radians.
        /// </summary>
        /// <value></value>
        public FixedPoint LowerLimit {
            get { return _lowerAngle; }
            set {
                if (_lowerAngle != value) {
                    WakeBodies();
                    _lowerAngle = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit in radians.
        /// </summary>
        /// <value></value>
        public FixedPoint UpperLimit {
            get { return _upperAngle; }
            set {
                if (_upperAngle != value) {
                    WakeBodies();
                    _upperAngle = value;
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
            if (lower != _lowerAngle || upper != _upperAngle) {
                WakeBodies();
                _upperAngle = upper;
                _lowerAngle = lower;
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
        /// Set the motor speed in radians per second.
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
        /// Set the maximum motor torque, usually in N-m.
        /// </summary>
        /// <value>The torque.</value>
        public FixedPoint MaxMotorTorque {
            set {
                WakeBodies();
                _maxMotorTorque = value;
            }
            get { return _maxMotorTorque; }
        }

        /// <summary>
        /// Get the current motor torque, usually in N-m.
        /// </summary>
        /// <value></value>
        public FixedPoint MotorImpulse {
            get { return _motorImpulse; }
            set {
                WakeBodies();
                _motorImpulse = value;
            }
        }

        public FixedPoint GetMotorTorque(FixedPoint inv_dt) {
            return inv_dt * _motorImpulse;
        }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            TSVector2 P = new TSVector2(_impulse.x, _impulse.y);
            return inv_dt * P;
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            return inv_dt * _impulse.z;
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

            //TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;

            //TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            m_rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            m_rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            FixedPoint mA = m_invMassA, mB = m_invMassB;
            FixedPoint iA = m_invIA, iB = m_invIB;

            bool fixedRotation = (iA + iB == 0.0f);

            m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
            m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
            m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
            m_mass.ex.y = m_mass.ey.x;
            m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
            m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
            m_mass.ex.z = m_mass.ez.x;
            m_mass.ey.z = m_mass.ez.y;
            m_mass.ez.z = iA + iB;

            m_motorMass = iA + iB;

            if (m_motorMass > 0.0f) {
                m_motorMass = 1.0f / m_motorMass;
            }

            if (_enableMotor == false || fixedRotation) {
                _motorImpulse = 0.0f;
            }

            if (_enableLimit && fixedRotation == false) {
                FixedPoint jointAngle = aB - aA - ReferenceAngle;
                if (TSMath.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.AngularSlop) {
                    _limitState = LimitState.Equal;
                }
                else if (jointAngle <= _lowerAngle) {
                    if (_limitState != LimitState.AtLower) {
                        _impulse.z = 0.0f;
                    }

                    _limitState = LimitState.AtLower;
                }
                else if (jointAngle >= _upperAngle) {
                    if (_limitState != LimitState.AtUpper) {
                        _impulse.z = 0.0f;
                    }

                    _limitState = LimitState.AtUpper;
                }
                else {
                    _limitState = LimitState.Inactive;
                    _impulse.z = 0.0f;
                }
            }
            else {
                _limitState = LimitState.Inactive;
            }

            if (Settings.EnableWarmstarting) {
                // Scale impulses to support a variable time step.
                _impulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_impulse.x, _impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + MotorImpulse + _impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + MotorImpulse + _impulse.z);
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
            //GABS: NOT A BOTTLENECK

            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            FixedPoint mA = m_invMassA, mB = m_invMassB;
            FixedPoint iA = m_invIA, iB = m_invIB;

            bool fixedRotation = (iA + iB == 0.0f);

            // Solve motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal && fixedRotation == false) {
                FixedPoint Cdot = wB - wA - _motorSpeed;
                FixedPoint impulse = m_motorMass * (-Cdot);
                FixedPoint oldImpulse = _motorImpulse;
                FixedPoint maxImpulse = data.step.dt * _maxMotorTorque;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false) {
                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                FixedPoint Cdot2 = wB - wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector impulse = -1 * m_mass.Solve33(Cdot);

                if (_limitState == LimitState.Equal) {
                    _impulse += impulse;
                }
                else if (_limitState == LimitState.AtLower) {
                    FixedPoint newImpulse = _impulse.z + impulse.z;
                    if (newImpulse < 0.0f) {
                        TSVector2 rhs = -Cdot1 + _impulse.z * new TSVector2(m_mass.ez.x, m_mass.ez.y);
                        TSVector2 reduced = m_mass.Solve22(rhs);
                        impulse.x = reduced.x;
                        impulse.y = reduced.y;
                        impulse.z = -_impulse.z;
                        _impulse.x += reduced.x;
                        _impulse.y += reduced.y;
                        _impulse.z = 0.0f;
                    }
                    else {
                        _impulse += impulse;
                    }
                }
                else if (_limitState == LimitState.AtUpper) {
                    FixedPoint newImpulse = _impulse.z + impulse.z;
                    if (newImpulse > 0.0f) {
                        TSVector2 rhs = -Cdot1 + _impulse.z * new TSVector2(m_mass.ez.x, m_mass.ez.y);
                        TSVector2 reduced = m_mass.Solve22(rhs);
                        impulse.x = reduced.x;
                        impulse.y = reduced.y;
                        impulse.z = -_impulse.z;
                        _impulse.x += reduced.x;
                        _impulse.y += reduced.y;
                        _impulse.z = 0.0f;
                    }
                    else {
                        _impulse += impulse;
                    }
                }

                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + impulse.z);
            }
            else {
                // Solve point-to-point constraint
                TSVector2 Cdot = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                TSVector2 impulse = m_mass.Solve22(-Cdot);

                _impulse.x += impulse.x;
                _impulse.y += impulse.y;

                vA -= mA * impulse;
                wA -= iA * MathUtils.Cross(m_rA, impulse);

                vB += mB * impulse;
                wB += iB * MathUtils.Cross(m_rB, impulse);
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

            FixedPoint angularError = 0.0f;
            FixedPoint positionError;

            bool fixedRotation = (m_invIA + m_invIB == 0.0f);

            // Solve angular limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false) {
                FixedPoint angle = aB - aA - ReferenceAngle;
                FixedPoint limitImpulse = 0.0f;

                if (_limitState == LimitState.Equal) {
                    // Prevent large angular corrections
                    FixedPoint C = MathUtils.Clamp(angle - _lowerAngle, -Settings.MaxAngularCorrection,
                        Settings.MaxAngularCorrection);
                    limitImpulse = -m_motorMass * C;
                    angularError = TSMath.Abs(C);
                }
                else if (_limitState == LimitState.AtLower) {
                    FixedPoint C = angle - _lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
                    limitImpulse = -m_motorMass * C;
                }
                else if (_limitState == LimitState.AtUpper) {
                    FixedPoint C = angle - _upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
                    limitImpulse = -m_motorMass * C;
                }

                aA -= m_invIA * limitImpulse;
                aB += m_invIB * limitImpulse;
            }

            // Solve point-to-point constraint.
            {
                qA.Set(aA);
                qB.Set(aB);
                TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
                TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);

                TSVector2 C = cB + rB - cA - rA;
                positionError = C.magnitude;

                FixedPoint mA = m_invMassA, mB = m_invMassB;
                FixedPoint iA = m_invIA, iB = m_invIB;

                Mat22 K = new Mat22();
                K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
                K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
                K.ey.x = K.ex.y;
                K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

                TSVector2 impulse = -K.Solve(C);

                cA -= mA * impulse;
                aA -= iA * MathUtils.Cross(rA, impulse);

                cB += mB * impulse;
                aB += iB * MathUtils.Cross(rB, impulse);
            }

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}