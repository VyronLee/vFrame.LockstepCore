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
    // 1-D rained system
    // m (v2 - v1) = lambda
    // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
    // x2 = x1 + h * v2

    // 1-D mass-damper-spring system
    // m (v2 - v1) + h * d * v2 + h * k *

    // C = norm(p2 - p1) - L
    // u = (p2 - p1) / norm(p2 - p1)
    // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    // J = [-u -cross(r1, u) u cross(r2, u)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

    /// <summary>
    /// A distance joint rains two points on two bodies
    /// to remain at a fixed distance from each other. You can view
    /// this as a massless, rigid rod.
    /// </summary>
    public class DistanceJoint : FarseerJoint
    {
        // Solver shared
        /// <summary>
        /// The local anchor point relative to bodyA's origin.
        /// </summary>
        public TSVector2 LocalAnchorA;

        /// <summary>
        /// The local anchor point relative to bodyB's origin.
        /// </summary>
        public TSVector2 LocalAnchorB;

        private FixedPoint _bias;
        private FixedPoint _gamma;
        private FixedPoint _impulse;

        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private TSVector2 _u;
        private TSVector2 m_rA;
        private TSVector2 m_rB;
        private TSVector2 m_localCenterA;
        private TSVector2 m_localCenterB;
        private FixedPoint m_invMassA;
        private FixedPoint m_invMassB;
        private FixedPoint m_invIA;
        private FixedPoint m_invIB;
        private FixedPoint _mass;

        internal DistanceJoint() {
            JointType = JointType.Distance;
        }

        /// <summary>
        /// This requires defining an
        /// anchor point on both bodies and the non-zero length of the
        /// distance joint. If you don't supply a length, the local anchor points
        /// is used so that the initial configuration can violate the constraint
        /// slightly. This helps when saving and loading a game.
        /// @warning Do not use a zero or short length.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="localAnchorA">The first body anchor</param>
        /// <param name="localAnchorB">The second body anchor</param>
        public DistanceJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB)
            : base(bodyA, bodyB) {
            JointType = JointType.Distance;

            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;

            TSVector2 d = WorldAnchorB - WorldAnchorA;
            Length = d.magnitude;
        }

        /// <summary>
        /// The natural length between the anchor points.
        /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
        /// </summary>
        public FixedPoint Length { get; set; }

        /// <summary>
        /// The mass-spring-damper frequency in Hertz. A value of 0
        /// disables softness.
        /// </summary>
        public FixedPoint Frequency { get; set; }

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public FixedPoint DampingRatio { get; set; }

        public override sealed TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override sealed TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// Get the reaction force given the inverse time step. Unit is N.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
        public override TSVector2 GetReactionForce(FixedPoint inv_dt) {
            TSVector2 F = (inv_dt * _impulse) * _u;
            return F;
        }

        /// <summary>
        /// Get the reaction torque given the inverse time step.
        /// Unit is N*m. This is always zero for a distance joint.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
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
            _u = cB + m_rB - cA - m_rA;

            // Handle singularity.
            FixedPoint length = _u.magnitude;
            if (length > Settings.LinearSlop) {
                _u *= 1.0f / length;
            }
            else {
                _u = TSVector2.zero;
            }

            FixedPoint crAu = MathUtils.Cross(m_rA, _u);
            FixedPoint crBu = MathUtils.Cross(m_rB, _u);
            FixedPoint invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

            // Compute the effective mass matrix.
            _mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (Frequency > 0.0f) {
                FixedPoint C = length - Length;

                // Frequency
                FixedPoint omega = 2.0f * Settings.Pi * Frequency;

                // Damping coefficient
                FixedPoint d = 2.0f * _mass * DampingRatio * omega;

                // Spring stiffness
                FixedPoint k = _mass * omega * omega;

                // magic formulas
                FixedPoint h = data.step.dt;
                _gamma = h * (d + h * k);
                _gamma = _gamma != 0.0f ? 1.0f / _gamma : 0.0f;
                _bias = C * h * k * _gamma;

                invMass += _gamma;
                _mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
            }
            else {
                _gamma = 0.0f;
                _bias = 0.0f;
            }

            if (Settings.EnableWarmstarting) {
                // Scale the impulse to support a variable time step.
                _impulse *= data.step.dtRatio;

                TSVector2 P = _impulse * _u;
                vA -= m_invMassA * P;
                wA -= m_invIA * MathUtils.Cross(m_rA, P);
                vB += m_invMassB * P;
                wB += m_invIB * MathUtils.Cross(m_rB, P);
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
            //GABS: NOT A BOTTLENECK

            TSVector2 vA = data.velocities[m_indexA].v;
            FixedPoint wA = data.velocities[m_indexA].w;
            TSVector2 vB = data.velocities[m_indexB].v;
            FixedPoint wB = data.velocities[m_indexB].w;

            // Cdot = dot(u, v + cross(w, r))
            TSVector2 vpA = vA + MathUtils.Cross(wA, m_rA);
            TSVector2 vpB = vB + MathUtils.Cross(wB, m_rB);
            FixedPoint Cdot = TSVector2.Dot(_u, vpB - vpA);

            FixedPoint impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
            _impulse += impulse;

            TSVector2 P = impulse * _u;
            vA -= m_invMassA * P;
            wA -= m_invIA * MathUtils.Cross(m_rA, P);
            vB += m_invMassB * P;
            wB += m_invIB * MathUtils.Cross(m_rB, P);

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            if (Frequency > 0.0f) {
                // There is no position correction for soft distance constraints.
                return true;
            }

            TSVector2 cA = data.positions[m_indexA].c;
            FixedPoint aA = data.positions[m_indexA].a;
            TSVector2 cB = data.positions[m_indexB].c;
            FixedPoint aB = data.positions[m_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - m_localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - m_localCenterB);
            TSVector2 u = cB + rB - cA - rA;

            FixedPoint length = u.magnitude;
            u.Normalize();
            FixedPoint C = length - Length;
            C = MathUtils.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

            FixedPoint impulse = -_mass * C;
            TSVector2 P = impulse * u;

            cA -= m_invMassA * P;
            aA -= m_invIA * MathUtils.Cross(rA, P);
            cB += m_invMassB * P;
            aB += m_invIB * MathUtils.Cross(rB, P);

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return TSMath.Abs(C) < Settings.LinearSlop;
        }
    }
}