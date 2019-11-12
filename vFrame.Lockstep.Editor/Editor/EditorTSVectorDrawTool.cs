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
            var lableRect = new Rect(initX + offset, position.y, 70, position.height);
            EditorGUI.LabelField(lableRect, label.text);
            var valRect = new Rect(initX + offset + lableWid, position.y, filedWid, position.height);
//        var fVal = EditorGUI.FloatField(valRect, property.intValue * 1.0f / LFloat.Precision);
//        property.intValue = (int) (fVal * LFloat.Precision);
            EditorGUI.FloatField(valRect, property.longValue);
            offset += filedWid + lableWid;
        }
    }
}
#endif