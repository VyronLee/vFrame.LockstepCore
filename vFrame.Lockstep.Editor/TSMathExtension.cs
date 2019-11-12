using UnityEngine;


namespace vFrame.Lockstep.Core.Editor
{
    public static partial class TSMathExtension
    {
        public static Vector2 ToVector2(this TSVector2 vec) {
            return new Vector2(vec.x.AsFloat(), vec.y.AsFloat());
        }

        public static TSVector2 ToTSVector2(this Vector2 vec) {
            return new TSVector2(vec.x, vec.y);
        }

        public static Vector3 ToVector3(this TSVector vec) {
            return new Vector3(vec.x.AsFloat(), vec.y.AsFloat(), vec.z.AsFloat());
        }

        public static TSVector ToTSVector(this Vector3 vec) {
            return new TSVector(
                vec.x,
                vec.y,
                vec.z);
        }
    }
}