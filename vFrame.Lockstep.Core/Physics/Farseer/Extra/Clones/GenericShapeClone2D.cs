namespace vFrame.Lockstep.Core.Physics2D {

    /**
    * @brief A generic 2D shape clone.
    **/
    public class GenericShapeClone2D {

        private FixedPoint _radius;
        private FixedPoint _2radius;

        private TSVector2 _position;

        private Vertices _vertices;
        private Vertices _normals;

        public void Clone(Shape sh) {
            this._radius = sh._radius;
            this._2radius = sh._2radius;

            if (sh is PolygonShape) {
                ClonePolygon((PolygonShape)sh);
            } else if (sh is CircleShape) {
                CloneCircle((CircleShape)sh);
            }
        }

        private void ClonePolygon(PolygonShape sh) {
            if (this._vertices == null) {
                this._vertices = new Vertices();
            } else {
                this._vertices.Clear();
            }

            if (this._normals == null) {
                this._normals = new Vertices();
            } else {
                this._normals.Clear();
            }

            this._vertices.AddRange(sh._vertices);
            this._normals.AddRange(sh._normals);
        }

        private void CloneCircle(CircleShape sh) {
            this._position = sh._position;
        }

        public void Restore(Shape sh) {
            sh._radius = this._radius;
            sh._2radius = this._2radius;

            if (sh is PolygonShape) {
                RestorePolygon((PolygonShape)sh);
            } else if (sh is CircleShape) {
                RestoreCircle((CircleShape)sh);
            }
        }

        private void RestorePolygon(PolygonShape sh) {
            sh._vertices.Clear();
            sh._vertices.AddRange(this._vertices);

            sh._normals.Clear();
            sh._normals.AddRange(this._normals);
        }

        private void RestoreCircle(CircleShape sh) {
            sh._position = this._position;
        }

    }

}