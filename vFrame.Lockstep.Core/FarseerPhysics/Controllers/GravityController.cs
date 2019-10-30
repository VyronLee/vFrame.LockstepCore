using System.Collections.Generic;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;

namespace vFrame.Lockstep.Core.FarseerPhysics.Controllers
{
    public enum GravityType
    {
        Linear,
        DistanceSquared
    }

    public class GravityController : Controller
    {
        public List<Body> Bodies = new List<Body>();
        public List<TSVector2> Points = new List<TSVector2>();

        public GravityController(FixedPoint strength)
            : base(ControllerType.GravityController) {
            Strength = strength;
            MaxRadius = FixedPoint.MaxValue;
        }

        public GravityController(FixedPoint strength, FixedPoint maxRadius, FixedPoint minRadius)
            : base(ControllerType.GravityController) {
            MinRadius = minRadius;
            MaxRadius = maxRadius;
            Strength = strength;
        }

        public FixedPoint MinRadius { get; set; }
        public FixedPoint MaxRadius { get; set; }
        public FixedPoint Strength { get; set; }
        public GravityType GravityType { get; set; }

        public override void Update(FixedPoint dt) {
            TSVector2 f = TSVector2.zero;

            foreach (Body body1 in World.BodyList) {
                if (!IsActiveOn(body1))
                    continue;

                foreach (Body body2 in Bodies) {
                    if (body1 == body2 || (body1.IsStatic && body2.IsStatic) || !body2.Enabled)
                        continue;

                    TSVector2 d = body2.WorldCenter - body1.WorldCenter;
                    FixedPoint r2 = d.LengthSquared();

                    if (r2 < Settings.Epsilon)
                        continue;

                    FixedPoint r = d.magnitude;

                    if (r >= MaxRadius || r <= MinRadius)
                        continue;

                    switch (GravityType) {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 / (FixedPoint) TSMath.Sqrt(r2) * body1.Mass * body2.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / r2 * body1.Mass * body2.Mass * d;
                            break;
                    }

                    body1.ApplyForce(ref f);
                    TSVector2.Negate(ref f, out f);
                    body2.ApplyForce(ref f);
                }

                foreach (TSVector2 point in Points) {
                    TSVector2 d = point - body1.Position;
                    FixedPoint r2 = d.LengthSquared();

                    if (r2 < Settings.Epsilon)
                        continue;

                    FixedPoint r = d.magnitude;

                    if (r >= MaxRadius || r <= MinRadius)
                        continue;

                    switch (GravityType) {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 / (FixedPoint) TSMath.Sqrt(r2) * body1.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / r2 * body1.Mass * d;
                            break;
                    }

                    body1.ApplyForce(ref f);
                }
            }
        }

        public void AddBody(Body body) {
            Bodies.Add(body);
        }

        public void AddPoint(TSVector2 point) {
            Points.Add(point);
        }
    }
}