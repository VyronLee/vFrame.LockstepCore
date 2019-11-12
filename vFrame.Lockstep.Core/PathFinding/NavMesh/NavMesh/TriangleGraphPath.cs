using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding
{
    public class TriangleGraphPath : DefaultGraphPath<IConnection<Triangle>>
    {
        /**
         * The start point when generating a point path for this triangle path
         */
        public TSVector start;

        /**
         * The end point when generating a point path for this triangle path
         */
        public TSVector end;

        /**
         * If the triangle path is empty, the point path will span this triangle
         */
        public Triangle startTri;

        /**
         * @return Last triangle in the path.
         */
        public Triangle GetEndTriangle() {
            return (GetCount() > 0) ? Get(GetCount() - 1).GetToNode() : startTri;
        }
    }
}