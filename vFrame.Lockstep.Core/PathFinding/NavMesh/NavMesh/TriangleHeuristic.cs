using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.PathFinding
{
    public class TriangleHeuristic : IHeuristic<Triangle>
    {
        private static TSVector A_AB = new TSVector();
        private static TSVector A_BC = new TSVector();
        private static TSVector A_CA = new TSVector();
        private static TSVector B_AB = new TSVector();
        private static TSVector B_BC = new TSVector();
        private static TSVector B_CA = new TSVector();

        public FixedPoint Estimate(Triangle node, Triangle endNode) {
            FixedPoint dst2;
            FixedPoint minDst2 = FixedPoint.MaxValue;
            A_AB = (node.a).Add(node.b) * FixedPoint.Half;
            A_AB = (node.b).Add(node.c) * FixedPoint.Half;
            A_AB = (node.c).Add(node.a) * FixedPoint.Half;

            B_AB = (endNode.a).Add(endNode.b) * FixedPoint.Half;
            B_BC = (endNode.b).Add(endNode.c) * FixedPoint.Half;
            B_CA = (endNode.c).Add(endNode.a) * FixedPoint.Half;

            if ((dst2 = A_AB.dst2(B_AB)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_AB.dst2(B_BC)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_AB.dst2(B_CA)) < minDst2)
                minDst2 = dst2;

            if ((dst2 = A_BC.dst2(B_AB)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_BC.dst2(B_BC)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_BC.dst2(B_CA)) < minDst2)
                minDst2 = dst2;

            if ((dst2 = A_CA.dst2(B_AB)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_CA.dst2(B_BC)) < minDst2)
                minDst2 = dst2;
            if ((dst2 = A_CA.dst2(B_CA)) < minDst2)
                minDst2 = dst2;

            return (FixedPoint) TSMath.Sqrt(minDst2);
        }
    }
}