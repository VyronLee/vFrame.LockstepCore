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
    // Limit:
    // C = norm(pB - pA) - L
    // u = (pB - pA) / norm(pB - pA)
    // Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    // J = [-u -cross(rA, u) u cross(rB, u)]
    // K = J * invM * JT
    //   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

    /// <summary>
    /// A rope joint enforces a maximum distance between two points
    /// on two bodies. It has no other effect.
    /// Warning: if you attempt to change the maximum length during
    /// the simulation you will get some non-physical behavior.
    /// A model that would allow you to dynamically modify the length
    /// would have some sponginess, so I chose not to implement it
    /// that way. See b2DistanceJoint if you want to dynamically
    /// control length.
    /// </summary>
    public class RopeJoint : FarseerJoint
    {
        // Solver shared
        private FixedPoint _impulse;
        private FixedPoint _length;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FixedPoint _invMassA;
        private FixedPoint _invMassB;
        private FixedPoint _invIA;
        private FixedPoint _invIB;
        private FixedPoint _mass;
        private TSVector2 _rA, _rB;
        private LimitState _state;
        private TSVector2 _u;

        internal RopeJoint() {
            JointType = JointType.Rope;
        }

        public RopeJoint(Body bodyA, Body bodyB, TSVector2 localAnchorA, TSVector2 localAnchorB)
            : base(bodyA, bodyB) {
            JointType = JointType.Rope;
            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;

            //FPE: Setting default MaxLength
            TSVector2 d = WorldAnchorB - WorldAnchorA;
            MaxLength = d.magnitude;
        }

        /// Get the maximum length of the rope.
        public FixedPoint MaxLength { get; set; }

        public LimitState State {
            get { return _state; }
        }

        public TSVector2 LocalAnchorA { get; set; }

        public TSVector2 LocalAnchorB { get; set; }

        public override sealed TSVector2 WorldAnchorA {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
        }

        public override sealed TSVector2 WorldAnchorB {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override TSVector2 GetReactionForce(FixedPoint invDt) {
            return (invDt * _impulse) * _u;
        }

        public override FixedPoint GetReactionTorque(FixedPoint invDt) {
            return 0;
        }

        internal override void InitVelocityConstraints(ref SolverData data) {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA.Sweep.LocalCenter;
            _localCenterB = BodyB.Sweep.LocalCenter;
            _invMassA = BodyA.InvMass;
            _invMassB = BodyB.InvMass;
            _invIA = BodyA.InvI;
            _invIB = BodyB.InvI;

            TSVector2 cA = data.positions[_indexA].c;
            FixedPoint aA = data.positions[_indexA].a;
            TSVector2 vA = data.velocities[_indexA].v;
            FixedPoint wA = data.velocities[_indexA].w;

            TSVector2 cB = data.positions[_indexB].c;
            FixedPoint aB = data.positions[_indexB].a;
            TSVector2 vB = data.velocities[_indexB].v;
            FixedPoint wB = data.velocities[_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            _rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            _u = cB + _rB - cA - _rA;

            _length = _u.magnitude;

            FixedPoint C = _length - MaxLength;
            if (C > 0.0f) {
                _state = LimitState.AtUpper;
            }
            else {
                _state = LimitState.Inactive;
            }

            if (_length > Settings.LinearSlop) {
                _u *= 1.0f / _length;
            }
            else {
                _u = TSVector2.zero;
                _mass = 0.0f;
                _impulse = 0.0f;
                return;
            }

            // Compute effective mass.
            FixedPoint crA = MathUtils.Cross(_rA, _u);
            FixedPoint crB = MathUtils.Cross(_rB, _u);
            FixedPoint invMass = _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

            _mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (Settings.EnableWarmstarting) {
                // Scale the impulse to support a variable time step.
                _impulse *= data.step.dtRatio;

                TSVector2 P = _impulse * _u;
                vA -= _invMassA * P;
                wA -= _invIA * MathUtils.Cross(_rA, P);
                vB += _invMassB * P;
                wB += _invIB * MathUtils.Cross(_rB, P);
            }
            else {
                _impulse = 0.0f;
            }


            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data) {
            TSVector2 vA = data.velocities[_indexA].v;
            FixedPoint wA = data.velocities[_indexA].w;
            TSVector2 vB = data.velocities[_indexB].v;
            FixedPoint wB = data.velocities[_indexB].w;


            // Cdot = dot(u, v + cross(w, r))
            TSVector2 vpA = vA + MathUtils.Cross(wA, _rA);
            TSVector2 vpB = vB + MathUtils.Cross(wB, _rB);
            FixedPoint C = _length - MaxLength;
            FixedPoint Cdot = TSVector2.Dot(_u, vpB - vpA);

            // Predictive constraint.
            if (C < 0.0f) {
                Cdot += data.step.inv_dt * C;
            }

            FixedPoint impulse = -_mass * Cdot;
            FixedPoint oldImpulse = _impulse;
            _impulse = TSMath.Min(0.0f, _impulse + impulse);
            impulse = _impulse - oldImpulse;

            TSVector2 P = impulse * _u;
            vA -= _invMassA * P;
            wA -= _invIA * MathUtils.Cross(_rA, P);
            vB += _invMassB * P;
            wB += _invIB * MathUtils.Cross(_rB, P);

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data) {
            TSVector2 cA = data.positions[_indexA].c;
            FixedPoint aA = data.positions[_indexA].a;
            TSVector2 cB = data.positions[_indexB].c;
            FixedPoint aB = data.positions[_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            TSVector2 u = cB + rB - cA - rA;

            FixedPoint length = u.magnitude;
            u.Normalize();

            FixedPoint C = length - MaxLength;

            C = MathUtils.Clamp(C, 0.0f, Settings.MaxLinearCorrection);

            FixedPoint impulse = -_mass * C;
            TSVector2 P = impulse * u;

            cA -= _invMassA * P;
            aA -= _invIA * MathUtils.Cross(rA, P);
            cB += _invMassB * P;
            aB += _invIB * MathUtils.Cross(rB, P);

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return length - MaxLength < Settings.LinearSlop;
        }
    }
}