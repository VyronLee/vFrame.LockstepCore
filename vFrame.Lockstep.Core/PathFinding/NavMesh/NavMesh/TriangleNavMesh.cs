using System.Collections.Generic;
using vFrame.Core.ThirdParty.LitJson;
using vFrame.Lockstep.Core.PathFinding.NavMesh.BSP;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh
{
    public class TriangleNavMesh : NavMesh
    {
        public TriangleGraph _graph;
        private TriangleHeuristic _heuristic;
        public IndexedAStarPathFinder<Triangle> _pathFinder;

        public TriangleNavMesh(string navMeshStr) : this(navMeshStr, 1) {
        }

        public BspTree bspTree => _graph.bspTree;

        public TriangleNavMesh(string navMeshStr, int scale) {
            var data = JsonMapper.ToObject<TriangleData>(navMeshStr);
            _graph = new TriangleGraph(data, scale);
            _pathFinder = new IndexedAStarPathFinder<Triangle>(_graph, true);
            _heuristic = new TriangleHeuristic();
        }

        public TriangleGraphPath navMeshGraphPath = null;

        public List<TSVector> FindPath(TSVector fromPoint, TSVector toPoint, TrianglePointPath navMeshPointPath) {
            navMeshGraphPath = new TriangleGraphPath();
            var find = FindPath(fromPoint, toPoint, navMeshGraphPath);
            if (!find) return null;

            navMeshPointPath.CalculateForGraphPath(navMeshGraphPath, false);
            return navMeshPointPath.getVectors();
        }

        private bool FindPath(TSVector fromPoint, TSVector toPoint, TriangleGraphPath path) {
            path.Clear();
            var fromTriangle = GetTriangle(fromPoint);
            var toTriangle = GetTriangle(toPoint);
            if (_pathFinder.SearchPath(fromTriangle, toTriangle, _heuristic, path)) {
                path.start = fromPoint;
                path.end = toPoint;
                path.startTri = fromTriangle;
                return true;
            }

            return false;
        }

        public TriangleGraph GetGraph() {
            return _graph;
        }

        public TriangleHeuristic GetHeuristic() {
            return _heuristic;
        }

        public IndexedAStarPathFinder<Triangle> GetPathFinder() {
            return _pathFinder;
        }


        public Triangle GetTriangle(TSVector point) {
            return _graph.GetTriangle(point);
        }
    }
}