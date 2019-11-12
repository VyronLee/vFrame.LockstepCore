#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace vFrame.Lockstep.Core.Editor
{
    public static class EditorGUILayoutExt
    {
        public static FixedPoint FloatField(string label, FixedPoint value, params GUILayoutOption[] options) {
            return EditorGUILayout.FloatField(label, value.AsFloat(), options);
        }

        public static TSVector2 Vector2Field(string label, TSVector2 value, params GUILayoutOption[] options) {
            return EditorGUILayout.Vector2Field(label, value.ToVector2(), options).ToTSVector2();
        }

        public static TSVector Vector3Field(string label, TSVector value, params GUILayoutOption[] options) {
            return EditorGUILayout.Vector3Field(label, value.ToVector3(), options).ToTSVector();
        }
    }
}
#endif