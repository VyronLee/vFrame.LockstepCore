namespace vFrame.Lockstep.Core.Physics2D
{
    internal class Point
    {
        // Pointers to next and previous points in Monontone Mountain
        public Point Next, Prev;
        public FixedPoint X, Y;

        public Point(FixedPoint x, FixedPoint y) {
            X = x;
            Y = y;
            Next = null;
            Prev = null;
        }

        public static Point operator -(Point p1, Point p2) {
            return new Point(p1.X - p2.X, p1.Y - p2.Y);
        }

        public static Point operator +(Point p1, Point p2) {
            return new Point(p1.X + p2.X, p1.Y + p2.Y);
        }

        public static Point operator -(Point p1, FixedPoint f) {
            return new Point(p1.X - f, p1.Y - f);
        }

        public static Point operator +(Point p1, FixedPoint f) {
            return new Point(p1.X + f, p1.Y + f);
        }

        public FixedPoint Cross(Point p) {
            return X * p.Y - Y * p.X;
        }

        public FixedPoint Dot(Point p) {
            return X * p.X + Y * p.Y;
        }

        public bool Neq(Point p) {
            return p.X != X || p.Y != Y;
        }

        public FixedPoint Orient2D(Point pb, Point pc) {
            FixedPoint acx = X - pc.X;
            FixedPoint bcx = pb.X - pc.X;
            FixedPoint acy = Y - pc.Y;
            FixedPoint bcy = pb.Y - pc.Y;
            return acx * bcy - acy * bcx;
        }
    }
}