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
    // C = p2 - p1
    // Cdot = v2 - v1
    //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // C = angle2 - angle1 - referenceAngle
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    /// <summary>
    /// A weld joint essentially glues two bodies together. A weld joint may
    /// distort somewhat because the island constraint solver is approximate.
    /// </summary>
    public class WeldJoint : FarseerJoint
    {
        // Solver shared
        public TSVector2 LocalAnchorA;
        public TSVector2 LocalAnchorB;
        private TSVector _impulse;
        private FixedPoint m_gamma;

        protected FixedPoint m_frequencyHz = 0f;
        protected FixedPoint m_dampingRatio = 0f;
        private FixedPoint m_bias;

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
        private Mat33 _mass;

        internal WeldJoint() {
            JointType = JointType.Weld;
        }

        /// <summary>
        /// You need to specify a local anchor point
        /// where they are attached and the relative body angle. The position
        /// of the anchor point is important for computing the reaction torque.
        /// You can change the anchor points relative to bodyA or bodyB by changing LocalAnchorA
        /// and/or LocalAnchorB.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="localAnchorA">The first body anchor.</param>
        /// <param name="localAnchorB">The second body anchor.</param>
        public WeldJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB)
            : base(bodyA, bodyB) {
            JointType = JointType.Weld;

            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;
            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;
        }

        public override TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The body2 angle minus body1 angle in the reference state (radians).
        /// </summary>
        public FixedPoint ReferenceAngle { get; private set; }

        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            return inv_dt * new TSVector2(_impulse.x, _impulse.y);
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

            Mat33 K = new Mat33();
            K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
            K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
            K.ez.x = -m_rA.y * iA - m_rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
            K.ez.y = m_rA.x * iA + m_rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (m_frequencyHz > 0.0f) {
                K.GetInverse22(ref _mass);

                FixedPoint invM = iA + iB;
                FixedPoint m = invM > 0.0f ? 1.0f / invM : 0.0f;

                FixedPoint C = aB - aA - ReferenceAngle;

                // Frequency
                FixedPoint omega = 2.0f * Settings.Pi * m_frequencyHz;

                // Damping coefficient
                FixedPoint d = 2.0f * m * m_dampingRatio * omega;

                // Spring stiffness
                FixedPoint k = m * omega * omega;

                // magic formulas
                FixedPoint h = data.step.dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invM += m_gamma;
                _mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else {
                K.GetSymInverse33(ref _mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }

            if (Settings.EnableWarmstarting) {
                // Scale impulses to support a variable time step.
                _impulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_impulse.x, _impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + _impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + _impulse.z);
            }
            else {
                _impulse = TSVector.zero;
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

            if (m_frequencyHz > 0.0f) {
                FixedPoint Cdot2 = wB - wA;

                FixedPoint impulse2 = -_mass.ez.z * (Cdot2 + m_bias + m_gamma * _impulse.z);
                _impulse.z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);

                TSVector2 impulse1 = -MathUtils.Mul22(_mass, Cdot1);
                _impulse.x += impulse1.x;
                _impulse.y += impulse1.y;

                TSVector2 P = impulse1;

                vA -= mA * P;
                wA -= iA * MathUtils.Cross(m_rA, P);

                vB += mB * P;
                wB += iB * MathUtils.Cross(m_rB, P);
            }
            else {
                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                FixedPoint Cdot2 = wB - wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector impulse = -1 * MathUtils.Mul(_mass, Cdot);
                _impulse += impulse;

                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + impulse.z);
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

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);

            FixedPoint positionError, angularError;

            Mat33 K = new Mat33();
            K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
            K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
            K.ez.x = -rA.y * iA - rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
            K.ez.y = rA.x * iA + rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (m_frequencyHz > 0.0f) {
                TSVector2 C1 = cB + rB - cA - rA;

                positionError = C1.magnitude;
                angularError = 0.0f;

                TSVector2 P = -K.Solve22(C1);

                cA -= mA * P;
                aA -= iA * MathUtils.Cross(rA, P);

                cB += mB * P;
                aB += iB * MathUtils.Cross(rB, P);
            }
            else {
                TSVector2 C1 = cB + rB - cA - rA;
                FixedPoint C2 = aB - aA - ReferenceAngle;

                positionError = C1.magnitude;
                angularError = TSMath.Abs(C2);

                TSVector C = new TSVector(C1.x, C1.y, C2);

                TSVector impulse = -1 * K.Solve33(C);
                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                cA -= mA * P;
                aA -= iA * (MathUtils.Cross(rA, P) + impulse.z);

                cB += mB * P;
                aB += iB * (MathUtils.Cross(rB, P) + impulse.z);
            }

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}