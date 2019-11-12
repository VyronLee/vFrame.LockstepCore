#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace vFrame.Lockstep.Core.Editor
{
    [CustomPropertyDrawer(typeof(FixedPoint))]
    public class EditorLFloat : UnityEditor.PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            var xProperty = property.FindPropertyRelative("_serializedValue");

            float LabelWidth = EditorGUIUtility.labelWidth - EditorTSVectorDrawTool.LableWidthOffset;
            float lableWid = EditorTSVectorDrawTool.LableWid;
            var labelRect = new Rect(position.x, position.y, LabelWidth, position.height);
            EditorGUI.LabelField(labelRect, label);

            float filedWid = (position.width - LabelWidth);
            float initX = position.x + LabelWidth;
            var valRect = new Rect(initX, position.y, filedWid, position.height);
            EditorGUI.FloatField(valRect, xProperty.longValue);
        }
    }
}
#endif