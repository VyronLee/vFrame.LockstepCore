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
    // Gear Joint:
    // C0 = (coordinate1 + ratio * coordinate2)_initial
    // C = (coordinate1 + ratio * coordinate2) - C0 = 0
    // J = [J1 ratio * J2]
    // K = J * invM * JT
    //   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
    //
    // Revolute:
    // coordinate = rotation
    // Cdot = angularVelocity
    // J = [0 0 1]
    // K = J * invM * JT = invI
    //
    // Prismatic:
    // coordinate = dot(p - pg, ug)
    // Cdot = dot(v + cross(w, r), ug)
    // J = [ug cross(r, ug)]
    // K = J * invM * JT = invMass + invI * cross(r, ug)^2

    /// <summary>
    /// A gear joint is used to connect two joints together. Either joint
    /// can be a revolute or prismatic joint. You specify a gear ratio
    /// to bind the motions together:
    /// coordinate1 + ratio * coordinate2 = ant
    /// The ratio can be negative or positive. If one joint is a revolute joint
    /// and the other joint is a prismatic joint, then the ratio will have units
    /// of length or units of 1/length.
    /// @warning You have to manually destroy the gear joint if joint1 or joint2
    /// is destroyed.
    /// </summary>
    public class GearJoint : FarseerJoint
    {
        //private FarseerJoint m_joint1;
        //private FarseerJoint m_joint2;

        private JointType m_typeA;
        private JointType m_typeB;

        // Body A is connected to body C
        // Body B is connected to body D
        private Body m_bodyC;
        private Body m_bodyD;

        // Solver shared
        private TSVector2 m_localAnchorA;
        private TSVector2 m_localAnchorB;
        private TSVector2 m_localAnchorC;
        private TSVector2 m_localAnchorD;

        private TSVector2 m_localAxisC;
        private TSVector2 m_localAxisD;

        private FixedPoint m_referenceAngleA;
        private FixedPoint m_referenceAngleB;

        private FixedPoint m_constant;
        private FixedPoint _ratio;

        private FixedPoint m_impulse;

        // Solver temp
        private int m_indexA, m_indexB, m_indexC, m_indexD;
        private TSVector2 m_lcA, m_lcB, m_lcC, m_lcD;
        private FixedPoint m_mA, m_mB, m_mC, m_mD;
        private FixedPoint m_iA, m_iB, m_iC, m_iD;
        private TSVector2 m_JvAC, m_JvBD;
        private FixedPoint m_JwA, m_JwB, m_JwC, m_JwD;
        private FixedPoint m_mass;

        /// <summary>
        /// Requires two existing revolute or prismatic joints (any combination will work).
        /// The provided joints must attach a dynamic body to a static body.
        /// </summary>
        /// <param name="jointA">The first joint.</param>
        /// <param name="jointB">The second joint.</param>
        /// <param name="ratio">The ratio.</param>
        public GearJoint(FarseerJoint jointA, FarseerJoint jointB, FixedPoint ratio)
            : base(jointA.BodyA, jointA.BodyB) {
            JointType = JointType.Gear;
            JointA = jointA;
            JointB = jointB;
            Ratio = ratio;

            m_typeA = jointA.JointType;
            m_typeB = jointB.JointType;

            // Make sure its the right kind of joint
            Debug.Assert(m_typeA == JointType.Revolute || m_typeA == JointType.Prismatic ||
                         m_typeA == JointType.FixedRevolute || m_typeA == JointType.FixedPrismatic);
            Debug.Assert(m_typeB == JointType.Revolute || m_typeB == JointType.Prismatic ||
                         m_typeB == JointType.FixedRevolute || m_typeB == JointType.FixedPrismatic);

            FixedPoint coordinateA = 0.0f, coordinateB = 0.0f;

            m_bodyC = JointA.BodyA;
            BodyA = JointA.BodyB;

            // Get geometry of joint1
            Transform xfA = BodyA.Xf;
            FixedPoint aA = BodyA.Sweep.A;
            Transform xfC = m_bodyC.Xf;
            FixedPoint aC = m_bodyC.Sweep.A;

            if (m_typeA == JointType.Revolute) {
                RevoluteJoint revolute = (RevoluteJoint) jointA;
                m_localAnchorC = revolute.LocalAnchorA;
                m_localAnchorA = revolute.LocalAnchorB;
                m_referenceAngleA = revolute.ReferenceAngle;
                m_localAxisC = TSVector2.zero;

                coordinateA = aA - aC - m_referenceAngleA;
            }
            else {
                PrismaticJoint prismatic = (PrismaticJoint) jointA;
                m_localAnchorC = prismatic.LocalAnchorA;
                m_localAnchorA = prismatic.LocalAnchorB;
                m_referenceAngleA = prismatic.ReferenceAngle;
                m_localAxisC = prismatic.LocalXAxisA;

                TSVector2 pC = m_localAnchorC;
                TSVector2 pA = MathUtils.MulT(xfC.q, MathUtils.Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
                coordinateA = TSVector2.Dot(pA - pC, m_localAxisC);
            }

            m_bodyD = JointB.BodyA;
            BodyB = JointB.BodyB;

            // Get geometry of joint2
            Transform xfB = BodyB.Xf;
            FixedPoint aB = BodyB.Sweep.A;
            Transform xfD = m_bodyD.Xf;
            FixedPoint aD = m_bodyD.Sweep.A;

            if (m_typeB == JointType.Revolute) {
                RevoluteJoint revolute = (RevoluteJoint) jointB;
                m_localAnchorD = revolute.LocalAnchorA;
                m_localAnchorB = revolute.LocalAnchorB;
                m_referenceAngleB = revolute.ReferenceAngle;
                m_localAxisD = TSVector2.zero;

                coordinateB = aB - aD - m_referenceAngleB;
            }
            else {
                PrismaticJoint prismatic = (PrismaticJoint) jointB;
                m_localAnchorD = prismatic.LocalAnchorA;
                m_localAnchorB = prismatic.LocalAnchorB;
                m_referenceAngleB = prismatic.ReferenceAngle;
                m_localAxisD = prismatic.LocalXAxisA;

                TSVector2 pD = m_localAnchorD;
                TSVector2 pB = MathUtils.MulT(xfD.q, MathUtils.Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
                coordinateB = TSVector2.Dot(pB - pD, m_localAxisD);
            }

            _ratio = ratio;
            m_constant = coordinateA + _ratio * coordinateB;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(m_localAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(m_localAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }


        /// <summary>
        /// The gear ratio.
        /// </summary>
        public FixedPoint Ratio {
            get { return _ratio; }
            set {
                Debug.Assert(MathUtils.IsValid(value));
                _ratio = value;
            }
        }

        /// <summary>
        /// The first revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public FarseerJoint JointA { get; set; }

        /// <summary>
        /// The second revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public FarseerJoint JointB { get; set; }


        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            TSVector2 P = m_impulse * m_JvAC;
            return inv_dt * P;
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            FixedPoint L = m_impulse * m_JwA;
            return inv_dt * L;
        }

        internal override void InitVelocityConstraints(ref SolverData data) {
            m_indexA = BodyA.IslandIndex;
            m_indexB = BodyB.IslandIndex;
            m_indexC = m_bodyC.IslandIndex;
            m_indexD = m_bodyD.IslandIndex;
            m_lcA = BodyA.Sweep.LocalCenter;
            m_lcB = BodyB.Sweep.LocalCenter;
            m_lcC = m_bodyC.Sweep.LocalCenter;
            m_lcD = m_bodyD.Sweep.LocalCenter;
            m_mA = BodyA.InvMass;
            m_mB = BodyB.InvMass;
            m_mC = m_bodyC.InvMass;
            m_mD = m_bodyD.InvMass;
            m_iA = BodyA.InvI;
            m_iB = BodyB.InvI;
            m_iC = m_bodyC.InvI;
            m_iD = m_bodyD.InvI;

            //TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;

            //TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            //TSVector2 cC = data.positions[m_indexC].c;
            FixedPoint aC = data.positions[m_indexC].a;
            TSVector2 vC = data.velocities[m_indexC].v;
            FixedPoint wC = data.velocities[m_indexC].w;

            //TSVector2 cD = data.positions[m_indexD].c;
            FixedPoint aD = data.positions[m_indexD].a;
            TSVector2 vD = data.velocities[m_indexD].v;
            FixedPoint wD = data.velocities[m_indexD].w;

            Rot qA = new Rot(aA), qB = new Rot(aB), qC = new Rot(aC), qD = new Rot(aD);

            m_mass = 0.0f;

            if (m_typeA == JointType.Revolute) {
                m_JvAC = TSVector2.zero;
                m_JwA = 1.0f;
                m_JwC = 1.0f;
                m_mass += m_iA + m_iC;
            }
            else {
                TSVector2 u = MathUtils.Mul(qC, m_localAxisC);
                TSVector2 rC = MathUtils.Mul(qC, m_localAnchorC - m_lcC);
                TSVector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_lcA);
                m_JvAC = u;
                m_JwC = MathUtils.Cross(rC, u);
                m_JwA = MathUtils.Cross(rA, u);
                m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
            }

            if (m_typeB == JointType.Revolute) {
                m_JvBD = TSVector2.zero;
                m_JwB = _ratio;
                m_JwD = _ratio;
                m_mass += _ratio * _ratio * (m_iB + m_iD);
            }
            else {
                TSVector2 u = MathUtils.Mul(qD, m_localAxisD);
                TSVector2 rD = MathUtils.Mul(qD, m_localAnchorD - m_lcD);
                TSVector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_lcB);
                m_JvBD = _ratio * u;
                m_JwD = _ratio * MathUtils.Cross(rD, u);
                m_JwB = _ratio * MathUtils.Cross(rB, u);
                m_mass += _ratio * _ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
            }

            // Compute effective mass.
            m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;

            if (Settings.EnableWarmstarting) {
                vA += (m_mA * m_impulse) * m_JvAC;
                wA += m_iA * m_impulse * m_JwA;
                vB += (m_mB * m_impulse) * m_JvBD;
                wB += m_iB * m_impulse * m_JwB;
                vC -= (m_mC * m_impulse) * m_JvAC;
                wC -= m_iC * m_impulse * m_JwC;
                vD -= (m_mD * m_impulse) * m_JvBD;
                wD -= m_iD * m_impulse * m_JwD;
            }
            else {
                m_impulse = 0.0f;
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
            data.velocities[m_indexC].v = vC;
            data.velocities[m_indexC].w = wC;
            data.velocities[m_indexD].v = vD;
            data.velocities[m_indexD].w = wD;
        }

        internal override void SolveVelocityConstraints(ref SolverData data) {
            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;
            TSVector2 vC = data.velocities[m_indexC].v;
            FixedPoint wC = data.velocities[m_indexC].w;
            TSVector2 vD = data.velocities[m_indexD].v;
            FixedPoint wD = data.velocities[m_indexD].w;

            FixedPoint Cdot = TSVector2.Dot(m_JvAC, vA - vC) + TSVector2.Dot(m_JvBD, vB - vD);
            Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

            FixedPoint impulse = -m_mass * Cdot;
            m_impulse += impulse;

            vA += (m_mA * impulse) * m_JvAC;
            wA += m_iA * impulse * m_JwA;
            vB += (m_mB * impulse) * m_JvBD;
            wB += m_iB * impulse * m_JwB;
            vC -= (m_mC * impulse) * m_JvAC;
            wC -= m_iC * impulse * m_JwC;
            vD -= (m_mD * impulse) * m_JvBD;
            wD -= m_iD * impulse * m_JwD;

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
            data.velocities[m_indexC].v = vC;
            data.velocities[m_indexC].w = wC;
            data.velocities[m_indexD].v = vD;
            data.velocities[m_indexD].w = wD;
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;
            TSVector2 cC = data.positions[m_indexC].c;
            FixedPoint aC = data.positions[m_indexC].a;
            TSVector2 cD = data.positions[m_indexD].c;
            FixedPoint aD = data.positions[m_indexD].a;

            Rot qA = new Rot(aA), qB = new Rot(aB), qC = new Rot(aC), qD = new Rot(aD);

            FixedPoint linearError = 0.0f;

            FixedPoint coordinateA, coordinateB;

            TSVector2 JvAC, JvBD;
            FixedPoint JwA, JwB, JwC, JwD;
            FixedPoint mass = 0.0f;

            if (m_typeA == JointType.Revolute) {
                JvAC = TSVector2.zero;
                JwA = 1.0f;
                JwC = 1.0f;
                mass += m_iA + m_iC;

                coordinateA = aA - aC - m_referenceAngleA;
            }
            else {
                TSVector2 u = MathUtils.Mul(qC, m_localAxisC);
                TSVector2 rC = MathUtils.Mul(qC, m_localAnchorC - m_lcC);
                TSVector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_lcA);
                JvAC = u;
                JwC = MathUtils.Cross(rC, u);
                JwA = MathUtils.Cross(rA, u);
                mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

                TSVector2 pC = m_localAnchorC - m_lcC;
                TSVector2 pA = MathUtils.MulT(qC, rA + (cA - cC));
                coordinateA = TSVector2.Dot(pA - pC, m_localAxisC);
            }

            if (m_typeB == JointType.Revolute) {
                JvBD = TSVector2.zero;
                JwB = _ratio;
                JwD = _ratio;
                mass += _ratio * _ratio * (m_iB + m_iD);

                coordinateB = aB - aD - m_referenceAngleB;
            }
            else {
                TSVector2 u = MathUtils.Mul(qD, m_localAxisD);
                TSVector2 rD = MathUtils.Mul(qD, m_localAnchorD - m_lcD);
                TSVector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_lcB);
                JvBD = _ratio * u;
                JwD = _ratio * MathUtils.Cross(rD, u);
                JwB = _ratio * MathUtils.Cross(rB, u);
                mass += _ratio * _ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

                TSVector2 pD = m_localAnchorD - m_lcD;
                TSVector2 pB = MathUtils.MulT(qD, rB + (cB - cD));
                coordinateB = TSVector2.Dot(pB - pD, m_localAxisD);
            }

            FixedPoint C = (coordinateA + _ratio * coordinateB) - m_constant;

            FixedPoint impulse = 0.0f;
            if (mass > 0.0f) {
                impulse = -C / mass;
            }

            cA += m_mA * impulse * JvAC;
            aA += m_iA * impulse * JwA;
            cB += m_mB * impulse * JvBD;
            aB += m_iB * impulse * JwB;
            cC -= m_mC * impulse * JvAC;
            aC -= m_iC * impulse * JwC;
            cD -= m_mD * impulse * JvBD;
            aD -= m_iD * impulse * JwD;

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;
            data.positions[m_indexC].c = cC;
            data.positions[m_indexC].a = aC;
            data.positions[m_indexD].c = cD;
            data.positions[m_indexD].a = aD;

            // TODO_ERIN not implemented
            return linearError < Settings.LinearSlop;
        }
    }
}