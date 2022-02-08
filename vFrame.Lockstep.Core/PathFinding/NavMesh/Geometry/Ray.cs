namespace vFrame.Lockstep.Core.PathFinding.NavMesh.Geometry;

public class Ray
{
    private static long serialVersionUID = -620692054835390878L;

    public TSVector direction = new(); //
    public TSVector origin = new(); //

    public Ray() {
    }

    public Ray(TSVector origin, TSVector direction) {
        this.origin.set(origin);
        this.direction.set(direction).nor();
    }

    /**
     * @return a copy of this ray.
     */
    public Ray cpy() {
        return new Ray(origin, direction);
    }


    public TSVector getEndPoint(TSVector _out, FixedPoint distance) {
        return _out.set(direction).scl(distance).Add(origin);
    }


    /**
     * {@inheritDoc}
     */
    public string toString() {
        return "ray [" + origin + ":" + direction + "]";
    }


    public Ray set(TSVector origin, TSVector direction) {
        this.origin.set(origin);
        this.direction.set(direction);
        return this;
    }

    public Ray set(FixedPoint x, FixedPoint y, FixedPoint z, FixedPoint dx, FixedPoint dy, FixedPoint dz) {
        origin.set(x, y, z);
        direction.set(dx, dy, dz);
        return this;
    }


    public Ray set(Ray ray) {
        origin.set(ray.origin);
        direction.set(ray.direction);
        return this;
    }
}