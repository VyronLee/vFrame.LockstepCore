using System.Collections.Generic;
using vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.BSP
{
    public class BspTree
    {
        public List<Triangle> allRawTriangle = new();
        public List<TriRef> allTriangle = new();

        public BspNode root;
        public static int maxDepth;
        public static int maxDepthNodeId;

        public void Init(List<Triangle> rawTriangles) {
            allRawTriangle = rawTriangles;
            foreach (var tri in rawTriangles)
                allTriangle.Add(new TriRef(tri));

            root = new BspNode();
            root.Init(allTriangle);
        }

        public int GetTriangle(TSVector pos) {
            var pos2d = pos.ToTSVector2XZ();
            return root.GetTriangle(pos2d);
        }
    }
}