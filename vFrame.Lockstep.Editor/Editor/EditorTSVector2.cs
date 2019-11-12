#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace vFrame.Lockstep.Core.Editor
{
    [CustomPropertyDrawer(typeof(TSVector2))]
    public class EditorTSVector2 : UnityEditor.PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            var xProperty = property.FindPropertyRelative("x");
            var yProperty = property.FindPropertyRelative("y");

            float LabelWidth = EditorGUIUtility.labelWidth - EditorTSVectorDrawTool.LableWidthOffset;
            float lableWid = EditorTSVectorDrawTool.LableWid;
            var labelRect = new Rect(position.x, position.y, LabelWidth, position.height);
            EditorGUI.LabelField(labelRect, label);

            float filedWid = (position.width - LabelWidth) / 2 - lableWid;
            float initX = position.x + LabelWidth;
            float offset = 0;
            EditorTSVectorDrawTool.DrawField(position, initX, ref offset, lableWid, filedWid, xProperty,
                new GUIContent("x:"));
            EditorTSVectorDrawTool.DrawField(position, initX, ref offset, lableWid, filedWid, yProperty,
                new GUIContent("y:"));
        }
    }
}
#endif