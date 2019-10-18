using System;
using System.Collections.Generic;
using TSRandom = vFrame.Lockstep.Core.TSRandom;

namespace vFrame.Lockstep.Core.Physics2D
{
    internal class PointGenerator
    {
        private static readonly TSRandom RNG = TSRandom.New(0);

        public static List<TriangulationPoint> UniformDistribution(int n, FixedPoint scale)
        {
            List<TriangulationPoint> points = new List<TriangulationPoint>();
            for (int i = 0; i < n; i++)
            {
                points.Add(new TriangulationPoint(scale*(FixedPoint.Half - RNG.NextFP()), scale*(FixedPoint.Half - RNG.NextFP())));
            }
            return points;
        }

        public static List<TriangulationPoint> UniformGrid(int n, FixedPoint scale)
        {
            FixedPoint x = 0;
            FixedPoint size = scale/n;
            FixedPoint halfScale = FixedPoint.Half*scale;

            List<TriangulationPoint> points = new List<TriangulationPoint>();
            for (int i = 0; i < n + 1; i++)
            {
                x = halfScale - i*size;
                for (int j = 0; j < n + 1; j++)
                {
                    points.Add(new TriangulationPoint(x, halfScale - j*size));
                }
            }
            return points;
        }
    }
}