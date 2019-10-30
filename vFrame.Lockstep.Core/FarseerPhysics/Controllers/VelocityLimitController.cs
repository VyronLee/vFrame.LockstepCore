using System.Collections.Generic;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Controllers
{
    /// <summary>
    /// Put a limit on the linear (translation - the movespeed) and angular (rotation) velocity
    /// of bodies added to this controller.
    /// </summary>
    public class VelocityLimitController : Controller
    {
        public bool LimitAngularVelocity = true;
        public bool LimitLinearVelocity = true;
        private List<Body> _bodies = new List<Body>();
        private FixedPoint _maxAngularSqared;
        private FixedPoint _maxAngularVelocity;
        private FixedPoint _maxLinearSqared;
        private FixedPoint _maxLinearVelocity;

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController"/> class.
        /// Sets the max linear velocity to Settings.MaxTranslation
        /// Sets the max angular velocity to Settings.MaxRotation
        /// </summary>
        public VelocityLimitController()
            : base(ControllerType.VelocityLimitController) {
            MaxLinearVelocity = Settings.MaxTranslation;
            MaxAngularVelocity = Settings.MaxRotation;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController"/> class.
        /// Pass in 0 or FixedPoint.MaxValue to disable the limit.
        /// maxAngularVelocity = 0 will disable the angular velocity limit.
        /// </summary>
        /// <param name="maxLinearVelocity">The max linear velocity.</param>
        /// <param name="maxAngularVelocity">The max angular velocity.</param>
        public VelocityLimitController(FixedPoint maxLinearVelocity, FixedPoint maxAngularVelocity)
            : base(ControllerType.VelocityLimitController) {
            if (maxLinearVelocity == 0 || maxLinearVelocity == FixedPoint.MaxValue)
                LimitLinearVelocity = false;

            if (maxAngularVelocity == 0 || maxAngularVelocity == FixedPoint.MaxValue)
                LimitAngularVelocity = false;

            MaxLinearVelocity = maxLinearVelocity;
            MaxAngularVelocity = maxAngularVelocity;
        }

        /// <summary>
        /// Gets or sets the max angular velocity.
        /// </summary>
        /// <value>The max angular velocity.</value>
        public FixedPoint MaxAngularVelocity {
            get { return _maxAngularVelocity; }
            set {
                _maxAngularVelocity = value;
                _maxAngularSqared = _maxAngularVelocity * _maxAngularVelocity;
            }
        }

        /// <summary>
        /// Gets or sets the max linear velocity.
        /// </summary>
        /// <value>The max linear velocity.</value>
        public FixedPoint MaxLinearVelocity {
            get { return _maxLinearVelocity; }
            set {
                _maxLinearVelocity = value;
                _maxLinearSqared = _maxLinearVelocity * _maxLinearVelocity;
            }
        }

        public override void Update(FixedPoint dt) {
            foreach (Body body in _bodies) {
                if (!IsActiveOn(body))
                    continue;

                if (LimitLinearVelocity) {
                    //Translation
                    // Check for large velocities.
                    FixedPoint translationX = dt * body.LinearVelocityInternal.x;
                    FixedPoint translationY = dt * body.LinearVelocityInternal.y;
                    FixedPoint result = translationX * translationX + translationY * translationY;

                    if (result > dt * _maxLinearSqared) {
                        FixedPoint sq = (FixedPoint) TSMath.Sqrt(result);

                        FixedPoint ratio = _maxLinearVelocity / sq;
                        body.LinearVelocityInternal.x *= ratio;
                        body.LinearVelocityInternal.y *= ratio;
                    }
                }

                if (LimitAngularVelocity) {
                    //Rotation
                    FixedPoint rotation = dt * body.AngularVelocityInternal;
                    if (rotation * rotation > _maxAngularSqared) {
                        FixedPoint ratio = _maxAngularVelocity / TSMath.Abs(rotation);
                        body.AngularVelocityInternal *= ratio;
                    }
                }
            }
        }

        public void AddBody(Body body) {
            _bodies.Add(body);
        }

        public void RemoveBody(Body body) {
            _bodies.Remove(body);
        }
    }
}