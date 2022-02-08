#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace vFrame.Lockstep.Core.Editor
{
    [CustomPropertyDrawer(typeof(FixedPoint))]
    public class EditorFixedPoint : UnityEditor.PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            var xProperty = property.FindPropertyRelative("_serializedValue");

            float labelWidth = EditorGUIUtility.labelWidth - EditorTSVectorDrawTool.LableWidthOffset;
            var labelRect = new Rect(position.x, position.y, labelWidth, position.height);
            EditorGUI.LabelField(labelRect, label);

            float filedWid = (position.width - labelWidth);
            float initX = position.x + labelWidth;
            var valRect = new Rect(initX, position.y, filedWid, position.height);

            var fVal = EditorGUI.FloatField(valRect, xProperty.longValue * 1.0f / FixedPoint.L_ONE);
            xProperty.longValue = (long) (fVal * FixedPoint.L_ONE);
        }
    }
}
#endif