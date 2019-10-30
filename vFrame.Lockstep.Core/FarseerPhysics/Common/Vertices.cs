using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using vFrame.Lockstep.Core.FarseerPhysics.Collision;

namespace vFrame.Lockstep.Core.FarseerPhysics.Common
{
#if !(XBOX360)
    [DebuggerDisplay("Count = {Count} Vertices = {ToString()}")]
#endif
    public class Vertices : List<TSVector2>
    {
        public Vertices() {
        }

        public Vertices(int capacity) {
            Capacity = capacity;
        }

        public Vertices(TSVector2[] vector2) {
            for (int i = 0; i < vector2.Length; i++) {
                Add(vector2[i]);
            }
        }

        public Vertices(IList<TSVector2> vertices) {
            for (int i = 0; i < vertices.Count; i++) {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// Nexts the index.
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int NextIndex(int index) {
            if (index == Count - 1) {
                return 0;
            }

            return index + 1;
        }

        public TSVector2 NextVertex(int index) {
            return this[NextIndex(index)];
        }

        /// <summary>
        /// Gets the previous index.
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int PreviousIndex(int index) {
            if (index == 0) {
                return Count - 1;
            }

            return index - 1;
        }

        public TSVector2 PreviousVertex(int index) {
            return this[PreviousIndex(index)];
        }

        /// <summary>
        /// Gets the signed area.
        /// </summary>
        /// <returns></returns>
        public FixedPoint GetSignedArea() {
            int i;
            FixedPoint area = 0;

            for (i = 0; i < Count; i++) {
                int j = (i + 1) % Count;
                area += this[i].x * this[j].y;
                area -= this[i].y * this[j].x;
            }

            area /= 2.0f;
            return area;
        }

        /// <summary>
        /// Gets the area.
        /// </summary>
        /// <returns></returns>
        public FixedPoint GetArea() {
            int i;
            FixedPoint area = 0;

            for (i = 0; i < Count; i++) {
                int j = (i + 1) % Count;
                area += this[i].x * this[j].y;
                area -= this[i].y * this[j].x;
            }

            area /= 2.0f;
            return (area < 0 ? -area : area);
        }

        /// <summary>
        /// Gets the centroid.
        /// </summary>
        /// <returns></returns>
        public TSVector2 GetCentroid() {
            // Same algorithm is used by Box2D

            TSVector2 c = TSVector2.zero;
            FixedPoint area = 0.0f;

            FixedPoint inv3 = 1.0f / 3.0f;
            TSVector2 pRef = TSVector2.zero;
            for (int i = 0; i < Count; ++i) {
                // Triangle vertices.
                TSVector2 p1 = pRef;
                TSVector2 p2 = this[i];
                TSVector2 p3 = i + 1 < Count ? this[i + 1] : this[0];

                TSVector2 e1 = p2 - p1;
                TSVector2 e2 = p3 - p1;

                FixedPoint D = MathUtils.Cross(e1, e2);

                FixedPoint triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                c += triangleArea * inv3 * (p1 + p2 + p3);
            }

            // Centroid
            c *= 1.0f / area;
            return c;
        }

        /// <summary>
        /// Gets the radius based on area.
        /// </summary>
        /// <returns></returns>
        public FixedPoint GetRadius() {
            FixedPoint area = GetSignedArea();

            FixedPoint radiusSqrd = (FixedPoint) area / MathHelper.Pi;
            if (radiusSqrd < 0) {
                radiusSqrd *= -1;
            }

            return (FixedPoint) TSMath.Sqrt(radiusSqrd);
        }

        /// <summary>
        /// Returns an AABB for vertex.
        /// </summary>
        /// <returns></returns>
        public AABB GetCollisionBox() {
            AABB aabb;
            TSVector2 lowerBound = new TSVector2(FixedPoint.MaxValue, FixedPoint.MaxValue);
            TSVector2 upperBound = new TSVector2(FixedPoint.MinValue, FixedPoint.MinValue);

            for (int i = 0; i < Count; ++i) {
                if (this[i].x < lowerBound.x) {
                    lowerBound.x = this[i].x;
                }

                if (this[i].x > upperBound.x) {
                    upperBound.x = this[i].x;
                }

                if (this[i].y < lowerBound.y) {
                    lowerBound.y = this[i].y;
                }

                if (this[i].y > upperBound.y) {
                    upperBound.y = this[i].y;
                }
            }

            aabb.LowerBound = lowerBound;
            aabb.UpperBound = upperBound;

            return aabb;
        }

        public void Translate(TSVector2 vector) {
            Translate(ref vector);
        }

        /// <summary>
        /// Translates the vertices with the specified vector.
        /// </summary>
        /// <param name="vector">The vector.</param>
        public void Translate(ref TSVector2 vector) {
            for (int i = 0; i < Count; i++)
                this[i] = TSVector2.Add(this[i], vector);
        }

        /// <summary>
        /// Scales the vertices with the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref TSVector2 value) {
            for (int i = 0; i < Count; i++)
                this[i] = TSVector2.Multiply(this[i], value);
        }

        /// <summary>
        /// Rotate the vertices with the defined value in radians.
        /// </summary>
        /// <param name="value">The amount to rotate by in radians.</param>
        public void Rotate(FixedPoint value) {
            TSMatrix4x4 rotationMatrix = TSMatrix4x4.RotateZ(value);

            for (int i = 0; i < Count; i++)
                this[i] = TSVector2.Transform(this[i], rotationMatrix);
        }

        /// <summary>
        /// Assuming the polygon is simple; determines whether the polygon is convex.
        /// NOTE: It will also return false if the input contains colinear edges.
        /// </summary>
        /// <returns>
        /// 	<c>true</c> if it is convex; otherwise, <c>false</c>.
        /// </returns>
        public bool IsConvex() {
            // Ensure the polygon is convex and the interior
            // is to the left of each edge.
            for (int i = 0; i < Count; ++i) {
                int i1 = i;
                int i2 = i + 1 < Count ? i + 1 : 0;
                TSVector2 edge = this[i2] - this[i1];

                for (int j = 0; j < Count; ++j) {
                    // Don't check vertices on the current edge.
                    if (j == i1 || j == i2) {
                        continue;
                    }

                    TSVector2 r = this[j] - this[i1];

                    FixedPoint s = edge.x * r.y - edge.y * r.x;

                    if (s <= 0.0f)
                        return false;
                }
            }

            return true;
        }

        public bool IsCounterClockWise() {
            //We just return true for lines
            if (Count < 3)
                return true;

            return (GetSignedArea() > 0.0f);
        }

        /// <summary>
        /// Forces counter clock wise order.
        /// </summary>
        public void ForceCounterClockWise() {
            if (!IsCounterClockWise()) {
                Reverse();
            }
        }

        /// <summary>
        /// Check for edge crossings
        /// </summary>
        /// <returns></returns>
        public bool IsSimple() {
            for (int i = 0; i < Count; ++i) {
                int iplus = (i + 1 > Count - 1) ? 0 : i + 1;
                TSVector2 a1 = new TSVector2(this[i].x, this[i].y);
                TSVector2 a2 = new TSVector2(this[iplus].x, this[iplus].y);
                for (int j = i + 1; j < Count; ++j) {
                    int jplus = (j + 1 > Count - 1) ? 0 : j + 1;
                    TSVector2 b1 = new TSVector2(this[j].x, this[j].y);
                    TSVector2 b2 = new TSVector2(this[jplus].x, this[jplus].y);

                    TSVector2 temp;

                    if (LineTools.LineIntersect2(a1, a2, b1, b2, out temp)) {
                        return false;
                    }
                }
            }

            return true;
        }

        //TODO: Test
        //Implementation found here: http://www.gamedev.net/community/forums/topic.asp?topic_id=548477
        public bool IsSimple2() {
            for (int i = 0; i < Count; ++i) {
                if (i < Count - 1) {
                    for (int h = i + 1; h < Count; ++h) {
                        // Do two vertices lie on top of one another?
                        if (this[i] == this[h]) {
                            return true;
                        }
                    }
                }

                int j = (i + 1) % Count;
                TSVector2 iToj = this[j] - this[i];
                TSVector2 iTojNormal = new TSVector2(iToj.y, -iToj.x);

                // i is the first vertex and j is the second
                int startK = (j + 1) % Count;
                int endK = (i - 1 + Count) % Count;
                endK += startK < endK ? 0 : startK + 1;
                int k = startK;
                TSVector2 iTok = this[k] - this[i];
                bool onLeftSide = TSVector2.Dot(iTok, iTojNormal) >= 0;
                TSVector2 prevK = this[k];
                ++k;
                for (; k <= endK; ++k) {
                    int modK = k % Count;
                    iTok = this[modK] - this[i];
                    if (onLeftSide != TSVector2.Dot(iTok, iTojNormal) >= 0) {
                        TSVector2 prevKtoK = this[modK] - prevK;
                        TSVector2 prevKtoKNormal = new TSVector2(prevKtoK.y, -prevKtoK.x);
                        if ((TSVector2.Dot(this[i] - prevK, prevKtoKNormal) >= 0) !=
                            (TSVector2.Dot(this[j] - prevK, prevKtoKNormal) >= 0)) {
                            return true;
                        }
                    }

                    onLeftSide = TSVector2.Dot(iTok, iTojNormal) > 0;
                    prevK = this[modK];
                }
            }

            return false;
        }

        // From Eric Jordan's convex decomposition library

        /// <summary>
        /// Checks if polygon is valid for use in Box2d engine.
        /// Last ditch effort to ensure no invalid polygons are
        /// added to world geometry.
        ///
        /// Performs a full check, for simplicity, convexity,
        /// orientation, minimum angle, and volume.  This won't
        /// be very efficient, and a lot of it is redundant when
        /// other tools in this section are used.
        /// </summary>
        /// <returns></returns>
        public bool CheckPolygon(out int error, out string errorMessage) {
            error = -1;
            errorMessage = null;

            if (Count < 3 || Count > Settings.MaxPolygonVertices) {
                error = 0;
            }

            if (!IsConvex()) {
                error = 1;
            }

            if (!IsSimple()) {
                error = 2;
            }

            if (GetArea() < Settings.Epsilon) {
                error = 3;
            }

            //Compute normals
            TSVector2[] normals = new TSVector2[Count];
            Vertices vertices = new Vertices(Count);
            for (int i = 0; i < Count; ++i) {
                vertices.Add(new TSVector2(this[i].x, this[i].y));
                int i1 = i;
                int i2 = i + 1 < Count ? i + 1 : 0;
                TSVector2 edge = new TSVector2(this[i2].x - this[i1].x, this[i2].y - this[i1].y);
                normals[i] = MathUtils.Cross(edge, 1.0f);
                normals[i].Normalize();
            }

            //Required side checks
            for (int i = 0; i < Count; ++i) {
                int iminus = (i == 0) ? Count - 1 : i - 1;

                //Parallel sides check
                FixedPoint cross = MathUtils.Cross(normals[iminus], normals[i]);
                cross = MathUtils.Clamp(cross, -1.0f, 1.0f);
                FixedPoint angle = (FixedPoint) TSMath.Asin(cross);
                if (angle <= Settings.AngularSlop) {
                    error = 4;
                    break;
                }

                //Too skinny check
                for (int j = 0; j < Count; ++j) {
                    if (j == i || j == (i + 1) % Count) {
                        continue;
                    }

                    FixedPoint s = TSVector2.Dot(normals[i], vertices[j] - vertices[i]);
                    if (s >= -Settings.LinearSlop) {
                        error = 5;
                    }
                }


                TSVector2 centroid = vertices.GetCentroid();
                TSVector2 n1 = normals[iminus];
                TSVector2 n2 = normals[i];
                TSVector2 v = vertices[i] - centroid;

                TSVector2 d = new TSVector2();
                d.x = TSVector2.Dot(n1, v); // - toiSlop;
                d.y = TSVector2.Dot(n2, v); // - toiSlop;

                // Shifting the edge inward by toiSlop should
                // not cause the plane to pass the centroid.
                if ((d.x < 0.0f) || (d.y < 0.0f)) {
                    error = 6;
                }
            }

            if (error != -1) {
                switch (error) {
                    case 0:
                        errorMessage = string.Format("Polygon error: must have between 3 and {0} vertices.",
                            Settings.MaxPolygonVertices);
                        break;
                    case 1:
                        errorMessage = "Polygon error: must be convex.";
                        break;
                    case 2:
                        errorMessage = "Polygon error: must be simple (cannot intersect itself).";
                        break;
                    case 3:
                        errorMessage = "Polygon error: area is too small.";
                        break;
                    case 4:
                        errorMessage = "Polygon error: sides are too close to parallel.";
                        break;
                    case 5:
                        errorMessage = "Polygon error: polygon is too thin.";
                        break;
                    case 6:
                        errorMessage = "Polygon error: core shape generation would move edge past centroid (too thin).";
                        break;
                    default:
                        errorMessage = "Polygon error: error " + error.ToString();
                        break;
                }
            }

            return error != -1;
        }

        public bool CheckPolygon() {
            string errorMessage;
            int errorCode;
            var result = CheckPolygon(out errorCode, out errorMessage);
            if (!result && errorMessage != null) {
                Debug.WriteLine(errorMessage);
            }

            return result;
        }

        // From Eric Jordan's convex decomposition library

        /// <summary>
        /// Trace the edge of a non-simple polygon and return a simple polygon.
        ///
        /// Method:
        /// Start at vertex with minimum y (pick maximum x one if there are two).
        /// We aim our "lastDir" vector at (1.0, 0)
        /// We look at the two rays going off from our start vertex, and follow whichever
        /// has the smallest angle (in -Pi . Pi) wrt lastDir ("rightest" turn)
        /// Loop until we hit starting vertex:
        /// We add our current vertex to the list.
        /// We check the seg from current vertex to next vertex for intersections
        /// - if no intersections, follow to next vertex and continue
        /// - if intersections, pick one with minimum distance
        /// - if more than one, pick one with "rightest" next point (two possibilities for each)
        /// </summary>
        /// <param name="verts">The vertices.</param>
        /// <returns></returns>
        public Vertices TraceEdge(Vertices verts) {
            PolyNode[] nodes = new PolyNode[verts.Count * verts.Count];
            //overkill, but sufficient (order of mag. is right)
            int nNodes = 0;

            //Add base nodes (raw outline)
            for (int i = 0; i < verts.Count; ++i) {
                TSVector2 pos = new TSVector2(verts[i].x, verts[i].y);
                nodes[i].Position = pos;
                ++nNodes;
                int iplus = (i == verts.Count - 1) ? 0 : i + 1;
                int iminus = (i == 0) ? verts.Count - 1 : i - 1;
                nodes[i].AddConnection(nodes[iplus]);
                nodes[i].AddConnection(nodes[iminus]);
            }

            //Process intersection nodes - horribly inefficient
            bool dirty = true;
            int counter = 0;
            while (dirty) {
                dirty = false;
                for (int i = 0; i < nNodes; ++i) {
                    for (int j = 0; j < nodes[i].NConnected; ++j) {
                        for (int k = 0; k < nNodes; ++k) {
                            if (k == i || nodes[k] == nodes[i].Connected[j]) continue;
                            for (int l = 0; l < nodes[k].NConnected; ++l) {
                                if (nodes[k].Connected[l] == nodes[i].Connected[j] ||
                                    nodes[k].Connected[l] == nodes[i]) continue;

                                //Check intersection
                                TSVector2 intersectPt;

                                bool crosses = LineTools.LineIntersect(nodes[i].Position,
                                    nodes[i].Connected[j].Position,
                                    nodes[k].Position, nodes[k].Connected[l].Position,
                                    out intersectPt);
                                if (crosses) {
                                    dirty = true;
                                    //Destroy and re-hook connections at crossing point
                                    PolyNode connj = nodes[i].Connected[j];
                                    PolyNode connl = nodes[k].Connected[l];
                                    nodes[i].Connected[j].RemoveConnection(nodes[i]);
                                    nodes[i].RemoveConnection(connj);
                                    nodes[k].Connected[l].RemoveConnection(nodes[k]);
                                    nodes[k].RemoveConnection(connl);
                                    nodes[nNodes] = new PolyNode(intersectPt);
                                    nodes[nNodes].AddConnection(nodes[i]);
                                    nodes[i].AddConnection(nodes[nNodes]);
                                    nodes[nNodes].AddConnection(nodes[k]);
                                    nodes[k].AddConnection(nodes[nNodes]);
                                    nodes[nNodes].AddConnection(connj);
                                    connj.AddConnection(nodes[nNodes]);
                                    nodes[nNodes].AddConnection(connl);
                                    connl.AddConnection(nodes[nNodes]);
                                    ++nNodes;
                                    goto SkipOut;
                                }
                            }
                        }
                    }
                }

                SkipOut:
                ++counter;
            }

            //Collapse duplicate points
            bool foundDupe = true;
            int nActive = nNodes;
            while (foundDupe) {
                foundDupe = false;
                for (int i = 0; i < nNodes; ++i) {
                    if (nodes[i].NConnected == 0) continue;
                    for (int j = i + 1; j < nNodes; ++j) {
                        if (nodes[j].NConnected == 0) continue;
                        TSVector2 diff = nodes[i].Position - nodes[j].Position;
                        if (diff.LengthSquared() <= Settings.Epsilon * Settings.Epsilon) {
                            if (nActive <= 3)
                                return new Vertices();

                            //printf("Found dupe, %d left\n",nActive);
                            --nActive;
                            foundDupe = true;
                            PolyNode inode = nodes[i];
                            PolyNode jnode = nodes[j];
                            //Move all of j's connections to i, and orphan j
                            int njConn = jnode.NConnected;
                            for (int k = 0; k < njConn; ++k) {
                                PolyNode knode = jnode.Connected[k];
                                Debug.Assert(knode != jnode);
                                if (knode != inode) {
                                    inode.AddConnection(knode);
                                    knode.AddConnection(inode);
                                }

                                knode.RemoveConnection(jnode);
                            }

                            jnode.NConnected = 0;
                        }
                    }
                }
            }

            //Now walk the edge of the list

            //Find node with minimum y value (max x if equal)
            FixedPoint minY = FixedPoint.MaxValue;
            FixedPoint maxX = -FixedPoint.MaxValue;
            int minYIndex = -1;
            for (int i = 0; i < nNodes; ++i) {
                if (nodes[i].Position.y < minY && nodes[i].NConnected > 1) {
                    minY = nodes[i].Position.y;
                    minYIndex = i;
                    maxX = nodes[i].Position.x;
                }
                else if (nodes[i].Position.y == minY && nodes[i].Position.x > maxX && nodes[i].NConnected > 1) {
                    minYIndex = i;
                    maxX = nodes[i].Position.x;
                }
            }

            TSVector2 origDir = new TSVector2(1.0f, 0.0f);
            TSVector2[] resultVecs = new TSVector2[4 * nNodes];
            // nodes may be visited more than once, unfortunately - change to growable array!
            int nResultVecs = 0;
            PolyNode currentNode = nodes[minYIndex];
            PolyNode startNode = currentNode;
            Debug.Assert(currentNode.NConnected > 0);
            PolyNode nextNode = currentNode.GetRightestConnection(origDir);
            if (nextNode == null) {
                Vertices vertices = new Vertices(nResultVecs);

                for (int i = 0; i < nResultVecs; ++i) {
                    vertices.Add(resultVecs[i]);
                }

                return vertices;
            }

            // Borked, clean up our mess and return
            resultVecs[0] = startNode.Position;
            ++nResultVecs;
            while (nextNode != startNode) {
                if (nResultVecs > 4 * nNodes) {
                    Debug.Assert(false);
                }

                resultVecs[nResultVecs++] = nextNode.Position;
                PolyNode oldNode = currentNode;
                currentNode = nextNode;
                nextNode = currentNode.GetRightestConnection(oldNode);
                if (nextNode == null) {
                    Vertices vertices = new Vertices(nResultVecs);
                    for (int i = 0; i < nResultVecs; ++i) {
                        vertices.Add(resultVecs[i]);
                    }

                    return vertices;
                }

                // There was a problem, so jump out of the loop and use whatever garbage we've generated so far
            }

            return new Vertices();
        }

        // Split up a vertices object with holes returned from the texture trace into
        // several vertices objects (one for the outline and one for each hole
        // outline and holes should have opposing winding orders (not fully tested, not efficient)
        public List<Vertices> SplitAtHoles() {
            List<Vertices> result = new List<Vertices>();
            List<TSVector2> duplicate = new List<TSVector2>();
            int holeCount = 0;
            int index = 0;
            bool ignoreNext = false;

            result.Add(new Vertices());

            // search for duplicate points and trace the polygon
            // point by point
            for (int i = 0; i < Count; ++i) {
                for (int j = i + 1; j < Count; ++j) {
                    if (this[i] == this[j]) {
                        duplicate.Add(this[i]);
                    }
                }

                if (ignoreNext) {
                    ignoreNext = false;
                }
                else {
                    result[index].Add(this[i]);
                    if (duplicate.Contains(this[i])) {
                        if (index == 0) // jump to new shape
                        {
                            holeCount++;
                            index = holeCount;
                            result.Add(new Vertices());
                        }
                        else // jump back to starting shape
                        {
                            index = 0;
                        }

                        ignoreNext = true;
                    }
                }
            }

            return result;
        }

        private class PolyNode
        {
            private const int MaxConnected = 32;

            /*
             * Given sines and cosines, tells if A's angle is less than B's on -Pi, Pi
             * (in other words, is A "righter" than B)
             */
            public PolyNode[] Connected = new PolyNode[MaxConnected];
            public int NConnected;
            public TSVector2 Position;

            public PolyNode(TSVector2 pos) {
                Position = pos;
                NConnected = 0;
            }

            private bool IsRighter(FixedPoint sinA, FixedPoint cosA, FixedPoint sinB, FixedPoint cosB) {
                if (sinA < 0) {
                    if (sinB > 0 || cosA <= cosB) return true;
                    else return false;
                }
                else {
                    if (sinB < 0 || cosA <= cosB) return false;
                    else return true;
                }
            }

            public void AddConnection(PolyNode toMe) {
                Debug.Assert(NConnected < MaxConnected);

                // Ignore duplicate additions
                for (int i = 0; i < NConnected; ++i) {
                    if (Connected[i] == toMe) return;
                }

                Connected[NConnected] = toMe;
                ++NConnected;
            }

            public void RemoveConnection(PolyNode fromMe) {
                bool isFound = false;
                int foundIndex = -1;
                for (int i = 0; i < NConnected; ++i) {
                    if (fromMe == Connected[i]) {
                        //.position == connected[i].position){
                        isFound = true;
                        foundIndex = i;
                        break;
                    }
                }

                Debug.Assert(isFound);
                --NConnected;
                for (int i = foundIndex; i < NConnected; ++i) {
                    Connected[i] = Connected[i + 1];
                }
            }

            public PolyNode GetRightestConnection(PolyNode incoming) {
                if (NConnected == 0) Debug.Assert(false); // This means the connection graph is inconsistent
                if (NConnected == 1) {
                    //b2Assert(false);
                    // Because of the possibility of collapsing nearby points,
                    // we may end up with "spider legs" dangling off of a region.
                    // The correct behavior here is to turn around.
                    return incoming;
                }

                TSVector2 inDir = Position - incoming.Position;

                FixedPoint inLength = inDir.magnitude;
                inDir.Normalize();

                Debug.Assert(inLength > Settings.Epsilon);

                PolyNode result = null;
                for (int i = 0; i < NConnected; ++i) {
                    if (Connected[i] == incoming) continue;
                    TSVector2 testDir = Connected[i].Position - Position;
                    FixedPoint testLengthSqr = testDir.LengthSquared();
                    testDir.Normalize();
                    Debug.Assert(testLengthSqr >= Settings.Epsilon * Settings.Epsilon);
                    FixedPoint myCos = TSVector2.Dot(inDir, testDir);
                    FixedPoint mySin = MathUtils.Cross(inDir, testDir);
                    if (result != null) {
                        TSVector2 resultDir = result.Position - Position;
                        resultDir.Normalize();
                        FixedPoint resCos = TSVector2.Dot(inDir, resultDir);
                        FixedPoint resSin = MathUtils.Cross(inDir, resultDir);
                        if (IsRighter(mySin, myCos, resSin, resCos)) {
                            result = Connected[i];
                        }
                    }
                    else {
                        result = Connected[i];
                    }
                }

                Debug.Assert(result != null);

                return result;
            }

            public PolyNode GetRightestConnection(TSVector2 incomingDir) {
                TSVector2 diff = Position - incomingDir;
                PolyNode temp = new PolyNode(diff);
                PolyNode res = GetRightestConnection(temp);
                Debug.Assert(res != null);
                return res;
            }
        }

        public override string ToString() {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < Count; i++) {
                builder.Append(this[i].ToString());
                if (i < Count - 1) {
                    builder.Append(" ");
                }
            }

            return builder.ToString();
        }

        /// <summary>
        /// Projects to axis.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="min">The min.</param>
        /// <param name="max">The max.</param>
        public void ProjectToAxis(ref TSVector2 axis, out FixedPoint min, out FixedPoint max) {
            // To project a point on an axis use the dot product
            FixedPoint dotProduct = TSVector2.Dot(axis, this[0]);
            min = dotProduct;
            max = dotProduct;

            for (int i = 0; i < Count; i++) {
                dotProduct = TSVector2.Dot(this[i], axis);
                if (dotProduct < min) {
                    min = dotProduct;
                }
                else {
                    if (dotProduct > max) {
                        max = dotProduct;
                    }
                }
            }
        }

        /// <summary>
        /// Winding number test for a point in a polygon.
        /// </summary>
        /// See more info about the algorithm here: http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm
        /// <param name="point">The point to be tested.</param>
        /// <returns>-1 if the winding number is zero and the point is outside
        /// the polygon, 1 if the point is inside the polygon, and 0 if the point
        /// is on the polygons edge.</returns>
        public int PointInPolygon(ref TSVector2 point) {
            // Winding number
            int wn = 0;

            // Iterate through polygon's edges
            for (int i = 0; i < Count; i++) {
                // Get points
                TSVector2 p1 = this[i];
                TSVector2 p2 = this[NextIndex(i)];

                // Test if a point is directly on the edge
                TSVector2 edge = p2 - p1;
                FixedPoint area = MathUtils.Area(ref p1, ref p2, ref point);
                if (area == 0f && TSVector2.Dot(point - p1, edge) >= 0f && TSVector2.Dot(point - p2, edge) <= 0f) {
                    return 0;
                }

                // Test edge for intersection with ray from point
                if (p1.y <= point.y) {
                    if (p2.y > point.y && area > 0f) {
                        ++wn;
                    }
                }
                else {
                    if (p2.y <= point.y && area < 0f) {
                        --wn;
                    }
                }
            }

            return (wn == 0 ? -1 : 1);
        }

        /// <summary>
        /// Compute the sum of the angles made between the test point and each pair of points making up the polygon.
        /// If this sum is 2pi then the point is an interior point, if 0 then the point is an exterior point.
        /// ref: http://ozviz.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/  - Solution 2
        /// </summary>
        public bool PointInPolygonAngle(ref TSVector2 point) {
            FixedPoint angle = 0;

            // Iterate through polygon's edges
            for (int i = 0; i < Count; i++) {
                // Get points
                TSVector2 p1 = this[i] - point;
                TSVector2 p2 = this[NextIndex(i)] - point;

                angle += MathUtils.VectorAngle(ref p1, ref p2);
            }

            if (TSMath.Abs(angle) < TSMath.Pi) {
                return false;
            }

            return true;
        }
    }
}