namespace vFrame.Lockstep.Core
{
    public class PhyUtil
    {
        public static TSVector PhyVed2ToVec3(TSVector2 vec2)
        {
            return new TSVector(vec2.x, 0, vec2.y);
        }

        public static TSVector PhyVed2ToVec3(TSVector2 vec2, FP y)
        {
            return new TSVector(vec2.x, y, vec2.y);
        }
    }
}