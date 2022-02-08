#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace vFrame.Lockstep.Core.Editor
{
    public static class EditorTSVectorDrawTool
    {
        public const float LableWidthOffset = 45;
        public const float LableWid = 20;

        public static void DrawField(Rect position, float initX, ref float offset, float lableWid, float filedWid,
            SerializedProperty property, GUIContent label) {
            var labelRect = new Rect(initX + offset, position.y, 70, position.height);
            EditorGUI.LabelField(labelRect, label.text);

            var valRect = new Rect(initX + offset + lableWid, position.y, filedWid, position.height);
            var fVal = EditorGUI.FloatField(valRect, property.longValue * 1.0f / FixedPoint.L_ONE);
            property.longValue = (long) (fVal * FixedPoint.L_ONE);

            offset += filedWid + lableWid;
        }
    }
}
#endif