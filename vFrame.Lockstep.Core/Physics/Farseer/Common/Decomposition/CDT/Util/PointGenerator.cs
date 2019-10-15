using System;
using System.Collections.Generic;
using FP = vFrame.Lockstep.Core.FP;
using TSRandom = vFrame.Lockstep.Core.TSRandom;

namespace vFrame.Lockstep.Core.Physics2D
{
    internal class PointGenerator
    {
        private static readonly TSRandom RNG = TSRandom.New(0);

        public static List<TriangulationPoint> UniformDistribution(int n, FP scale)
        {
            List<TriangulationPoint> points = new List<TriangulationPoint>();
            for (int i = 0; i < n; i++)
            {
                points.Add(new TriangulationPoint(scale*(FP.Half - RNG.NextFP()), scale*(FP.Half - RNG.NextFP())));
            }
            return points;
        }

        public static List<TriangulationPoint> UniformGrid(int n, FP scale)
        {
            FP x = 0;
            FP size = scale/n;
            FP halfScale = FP.Half*scale;

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