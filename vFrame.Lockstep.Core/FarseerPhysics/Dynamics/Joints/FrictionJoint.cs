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
    // Point-to-point constraint
    // Cdot = v2 - v1
    //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    /// <summary>
    /// Friction joint. This is used for top-down friction.
    /// It provides 2D translational friction and angular friction.
    /// </summary>
    public class FrictionJoint : FarseerJoint
    {
        public TSVector2 LocalAnchorA;
        public TSVector2 LocalAnchorB;

        // Solver shared
        private TSVector2 _linearImpulse;
        private FixedPoint _angularImpulse;


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
        private FixedPoint _angularMass;
        private Mat22 _linearMass;

        internal FrictionJoint() {
            JointType = JointType.Friction;
        }

        public FrictionJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB)
            : base(bodyA, bodyB) {
            JointType = JointType.Friction;
            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The maximum friction force in N.
        /// </summary>
        public FixedPoint MaxForce { get; set; }

        /// <summary>
        /// The maximum friction torque in N-m.
        /// </summary>
        public FixedPoint MaxTorque { get; set; }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            return inv_dt * _linearImpulse;
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            return inv_dt * _angularImpulse;
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

            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;

            FixedPoint aB = data.positions[m_indexB].a;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            // Compute the effective mass matrix.
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

            Mat22 K = new Mat22();
            K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
            K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

            _linearMass = K.Inverse;

            _angularMass = iA + iB;
            if (_angularMass > 0.0f) {
                _angularMass = 1.0f / _angularMass;
            }

            if (Settings.EnableWarmstarting) {
                // Scale impulses to support a variable time step.
                _linearImpulse *= data.step.dtRatio;
                _angularImpulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_linearImpulse.x, _linearImpulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + _angularImpulse);
                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + _angularImpulse);
            }
            else {
                _linearImpulse = TSVector2.zero;
                _angularImpulse = 0.0f;
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

            FixedPoint h = data.step.dt;

            // Solve angular friction
            {
                FixedPoint Cdot = wB - wA;
                FixedPoint impulse = -_angularMass * Cdot;

                FixedPoint oldImpulse = _angularImpulse;
                FixedPoint maxImpulse = h * MaxTorque;
                _angularImpulse = MathUtils.Clamp(_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                TSVector2 Cdot = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);

                TSVector2 impulse = -MathUtils.Mul(ref _linearMass, Cdot);
                TSVector2 oldImpulse = _linearImpulse;
                _linearImpulse += impulse;

                FixedPoint maxImpulse = h * MaxForce;

                if (_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                    _linearImpulse.Normalize();
                    _linearImpulse *= maxImpulse;
                }

                impulse = _linearImpulse - oldImpulse;

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
            return true;
        }
    }
}