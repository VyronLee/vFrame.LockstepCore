using System;
using System.Collections.Generic;
using System.Diagnostics;
using vFrame.Lockstep.Core.PathFinding.NavMesh.NavMesh;

namespace vFrame.Lockstep.Core.PathFinding.NavMesh.BSP;

public class TriRef
{
    public TSVector2 a;
    public TSVector2 b;

    public SplitPlane[] borders;
    public TSVector2 c;
    public int index; //正数为 原三角形，否则为
    public bool isSplit;

    public TriRef(List<TSVector2> vertexs, TriRef tri)
        : this(vertexs[0], vertexs[1], vertexs[2], tri) {
    }


    public TriRef(TSVector2 a, TSVector2 b, TSVector2 c, TriRef tri)
        : this(a, b, c, tri.index) {
        isSplit = true;
    }

    public TriRef(Triangle tri) : this(
        tri.a.ToTSVector2XZ(),
        tri.b.ToTSVector2XZ(),
        tri.c.ToTSVector2XZ(),
        tri.index) {
    }

    public TriRef(TSVector2 a, TSVector2 b, TSVector2 c, int idx) {
        this.a = a;
        this.b = b;
        this.c = c;
        index = idx;

        borders = new[] {
            new(a, b),
            new SplitPlane(b, c),
            new SplitPlane(c, a)
        };
        //check valid
        CheckValid();
    }

    public TSVector2 this[int index] {
        get {
            switch (index) {
                case 0:
                    return a;
                case 1:
                    return b;
                case 2:
                    return c;
                default:
                    throw new IndexOutOfRangeException("vector idx invalid" + index);
            }
        }
    }

    public bool Contain(TSVector2 pos) {
        var isRightA = TSVector2.Cross(b - a, pos - a) > 0;
        if (isRightA)
            return false;
        var isRightB = TSVector2.Cross(c - b, pos - b) > 0;
        if (isRightB)
            return false;
        var isRightC = TSVector2.Cross(a - c, pos - c) > 0;
        if (isRightC)
            return false;
        return true;
    }

    private void CheckValid() {
        for (var i = 0; i < 3; i++)
            if (borders[i].dir == TSVector2.zero)
                Debug.Assert(false);
    }
}