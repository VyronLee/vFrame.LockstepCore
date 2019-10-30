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
    // Pulley:
    // length1 = norm(p1 - s1)
    // length2 = norm(p2 - s2)
    // C0 = (length1 + ratio * length2)_initial
    // C = C0 - (length1 + ratio * length2)
    // u1 = (p1 - s1) / norm(p1 - s1)
    // u2 = (p2 - s2) / norm(p2 - s2)
    // Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
    // J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

    /// <summary>
    /// The pulley joint is connected to two bodies and two fixed ground points.
    /// The pulley supports a ratio such that:
    /// length1 + ratio * length2 <= constant
    /// Yes, the force transmitted is scaled by the ratio.
    /// Warning: the pulley joint can get a bit squirrelly by itself. They often
    /// work better when combined with prismatic joints. You should also cover the
    /// the anchor points with static shapes to prevent one side from going to
    /// zero length.
    /// </summary>
    public class PulleyJoint : FarseerJoint
    {
        /// <summary>
        /// Get the first ground anchor.
        /// </summary>
        /// <value></value>
        public TSVector2 GroundAnchorA;

        /// <summary>
        /// Get the second ground anchor.
        /// </summary>
        /// <value></value>
        public TSVector2 GroundAnchorB;

        // Solver shared
        public TSVector2 LocalAnchorA;
        public TSVector2 LocalAnchorB;

        private FixedPoint _impulse;
        protected FixedPoint _limitImpulse1 = 0f;
        protected FixedPoint _limitImpulse2 = 0f;
        private FixedPoint m_constant;

        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private TSVector2 m_uA;
        private TSVector2 m_uB;
        private TSVector2 m_rA;
        private TSVector2 m_rB;
        private TSVector2 m_localCenterA;
        private TSVector2 m_localCenterB;
        private FixedPoint m_invMassA;
        private FixedPoint m_invMassB;
        private FixedPoint m_invIA;
        private FixedPoint m_invIB;
        private FixedPoint m_mass;

        internal PulleyJoint() {
            JointType = JointType.Pulley;
        }

        /// <summary>
        /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
        /// This requires two ground anchors,
        /// two dynamic body anchor points, max lengths for each side,
        /// and a pulley ratio.
        /// </summary>
        /// <param name="bA">The first body.</param>
        /// <param name="bB">The second body.</param>
        /// <param name="groundA">The ground anchor for the first body.</param>
        /// <param name="groundB">The ground anchor for the second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second body anchor.</param>
        /// <param name="ratio">The ratio.</param>
        public PulleyJoint(Body bA, Body bB, TSVector2 groundA, TSVector2 groundB, TSVector2 anchorA, TSVector2 anchorB,
            FixedPoint ratio)
            : base(bA, bB) {
            JointType = JointType.Pulley;

            GroundAnchorA = groundA;
            GroundAnchorB = groundB;
            LocalAnchorA = anchorA;
            LocalAnchorB = anchorB;

            Debug.Assert(ratio != 0.0f);
            Debug.Assert(ratio > Settings.Epsilon);

            Ratio = ratio;

            TSVector2 dA = BodyA.GetWorldPoint(anchorA) - groundA;
            LengthA = dA.magnitude;

            TSVector2 dB = BodyB.GetWorldPoint(anchorB) - groundB;
            LengthB = dB.magnitude;

            m_constant = LengthA + ratio * LengthB;

            _impulse = 0.0f;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// Get the current length of the segment attached to body1.
        /// </summary>
        /// <value></value>
        public FixedPoint LengthA { get; set; }

        /// <summary>
        /// Get the current length of the segment attached to body2.
        /// </summary>
        /// <value></value>
        public FixedPoint LengthB { get; set; }

        public FixedPoint CurrentLengthA {
            get {
                TSVector2 p = BodyA.GetWorldPoint(LocalAnchorA);
                TSVector2 s = GroundAnchorA;
                TSVector2 d = p - s;
                return d.magnitude;
            }
        }

        public FixedPoint CurrentLengthB {
            get {
                TSVector2 p = BodyB.GetWorldPoint(LocalAnchorB);
                TSVector2 s = GroundAnchorB;
                TSVector2 d = p - s;
                return d.magnitude;
            }
        }

        /// <summary>
        /// Get the pulley ratio.
        /// </summary>
        /// <value></value>
        public FixedPoint Ratio { get; set; }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            TSVector2 P = _impulse * m_uB;
            return inv_dt * P;
        }

        public override FixedPoint GetReactionTorque(FixedPoint inv_dt) {
            return 0.0f;
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

            m_rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            m_rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);

            // Get the pulley axes.
            m_uA = cA + m_rA - GroundAnchorA;
            m_uB = cB + m_rB - GroundAnchorB;

            FixedPoint lengthA = m_uA.magnitude;
            FixedPoint lengthB = m_uB.magnitude;

            if (lengthA > 10.0f * Settings.LinearSlop) {
                m_uA *= 1.0f / lengthA;
            }
            else {
                m_uA = TSVector2.zero;
            }

            if (lengthB > 10.0f * Settings.LinearSlop) {
                m_uB *= 1.0f / lengthB;
            }
            else {
                m_uB = TSVector2.zero;
            }

            // Compute effective mass.
            FixedPoint ruA = MathUtils.Cross(m_rA, m_uA);
            FixedPoint ruB = MathUtils.Cross(m_rB, m_uB);

            FixedPoint mA = m_invMassA + m_invIA * ruA * ruA;
            FixedPoint mB = m_invMassB + m_invIB * ruB * ruB;

            m_mass = mA + Ratio * Ratio * mB;

            if (m_mass > 0.0f) {
                m_mass = 1.0f / m_mass;
            }

            if (Settings.EnableWarmstarting) {
                // Scale impulses to support variable time steps.
                _impulse *= data.step.dtRatio;

                // Warm starting.
                TSVector2 PA = -(_impulse + _limitImpulse1) * m_uA;
                TSVector2 PB = (-Ratio * _impulse - _limitImpulse2) * m_uB;
                vA += m_invMassA * PA;
                wA += m_invIA * MathUtils.Cross(m_rA, PA);
                vB += m_invMassB * PB;
                wB += m_invIB * MathUtils.Cross(m_rB, PB);
            }
            else {
                _impulse = 0.0f;
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

            TSVector2 vpA = vA + MathUtils.Cross(wA, m_rA);
            TSVector2 vpB = vB + MathUtils.Cross(wB, m_rB);

            FixedPoint Cdot = -TSVector2.Dot(m_uA, vpA) - Ratio * TSVector2.Dot(m_uB, vpB);
            FixedPoint impulse = -m_mass * Cdot;
            _impulse += impulse;

            TSVector2 PA = -impulse * m_uA;
            TSVector2 PB = -Ratio * impulse * m_uB;
            vA += m_invMassA * PA;
            wA += m_invIA * MathUtils.Cross(m_rA, PA);
            vB += m_invMassB * PB;
            wB += m_invIB * MathUtils.Cross(m_rB, PB);

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

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);

            // Get the pulley axes.
            TSVector2 uA = cA + rA - GroundAnchorA;
            TSVector2 uB = cB + rB - GroundAnchorB;

            FixedPoint lengthA = uA.magnitude;
            FixedPoint lengthB = uB.magnitude;

            if (lengthA > 10.0f * Settings.LinearSlop) {
                uA *= 1.0f / lengthA;
            }
            else {
                uA = TSVector2.zero;
            }

            if (lengthB > 10.0f * Settings.LinearSlop) {
                uB *= 1.0f / lengthB;
            }
            else {
                uB = TSVector2.zero;
            }

            // Compute effective mass.
            FixedPoint ruA = MathUtils.Cross(rA, uA);
            FixedPoint ruB = MathUtils.Cross(rB, uB);

            FixedPoint mA = m_invMassA + m_invIA * ruA * ruA;
            FixedPoint mB = m_invMassB + m_invIB * ruB * ruB;

            FixedPoint mass = mA + Ratio * Ratio * mB;

            if (mass > 0.0f) {
                mass = 1.0f / mass;
            }

            FixedPoint C = m_constant - lengthA - Ratio * lengthB;
            FixedPoint linearError = TSMath.Abs(C);

            FixedPoint impulse = -mass * C;

            TSVector2 PA = -impulse * uA;
            TSVector2 PB = -Ratio * impulse * uB;

            cA += m_invMassA * PA;
            aA += m_invIA * MathUtils.Cross(rA, PA);
            cB += m_invMassB * PB;
            aB += m_invIB * MathUtils.Cross(rB, PB);

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return linearError < Settings.LinearSlop;
        }
    }
}