using UnityEngine;
using vFrame.Lockstep.Core;

namespace vFrame.Lockstep.Core.Editor
{
    public static class TSVectorExtension
    {
        public static Vector3[] ToVecArray(this TSVector[] lVecs) {
            var vecs = new Vector3[lVecs.Length];
            for (int i = 0; i < lVecs.Length; i++) {
                vecs[i] = lVecs[i].ToVector3();
            }

            return vecs;
        }

        public static TSVector[] ToLVecArray(this Vector3[] lVecs) {
            var vecs = new TSVector[lVecs.Length];
            for (int i = 0; i < lVecs.Length; i++) {
                vecs[i] = lVecs[i].ToTSVector();
            }

            return vecs;
        }
    }
}