namespace vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh;

public class TriangleHeuristic : IHeuristic<Triangle>
{
    private static TSVector A_AB;
    private static readonly TSVector A_BC = new();
    private static readonly TSVector A_CA = new();
    private static TSVector B_AB;
    private static TSVector B_BC;
    private static TSVector B_CA;

    public FixedPoint Estimate(Triangle node, Triangle endNode) {
        FixedPoint dst2;
        var minDst2 = FixedPoint.MaxValue;
        A_AB = node.a.Add(node.b) * FixedPoint.Half;
        A_AB = node.b.Add(node.c) * FixedPoint.Half;
        A_AB = node.c.Add(node.a) * FixedPoint.Half;

        B_AB = endNode.a.Add(endNode.b) * FixedPoint.Half;
        B_BC = endNode.b.Add(endNode.c) * FixedPoint.Half;
        B_CA = endNode.c.Add(endNode.a) * FixedPoint.Half;

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

        return TSMath.Sqrt(minDst2);
    }
}