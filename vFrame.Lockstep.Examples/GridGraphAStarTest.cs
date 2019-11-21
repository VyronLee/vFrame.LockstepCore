using System;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.PathFinding;
using vFrame.Lockstep.Core.PathFinding.Grid;

namespace vFrame.Lockstep.Examples
{
    internal static class GridGraphAStarTest
    {
        private static readonly int[] grids = {
            1, 1, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 0, 0, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 0, 0, 0, 0, 0, 0, 1, 1,
            1, 0, 0, 0, 0, 1, 1, 0, 1, 1,
            1, 1, 0, 0, 0, 1, 1, 0, 0, 1,
            1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
            1, 1, 0, 1, 1, 0, 0, 0, 0, 1,
            1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        };

        public static void Main(string[] args) {
            var gridData = new GridData {cols = 10, rows = 10, start = TSVector2.zero, grids = grids, width = 10};
            var gridGraph = new GridGraph(gridData, 1);

            var finder = new IndexedAStarPathFinder<GridNode>(gridGraph);
            var outPath = new GridGraphPath();
            finder.SearchPath(gridGraph.GetNode(0, 0), gridGraph.GetNode(4, 5), new GridHeuristic(), outPath);

            Console.WriteLine("Search path finished: ");
            for (var i = 0; i < outPath.GetCount(); i++) {
                var node = outPath.Get(i).GetFromNode();
                Console.Write($"({node.xIndex}, {node.yIndex}) ");
            }

            var endNode = outPath.Get(outPath.GetCount() - 1).GetToNode();
            Console.Write($"({endNode.xIndex}, {endNode.yIndex}) ");
        }
    }
}