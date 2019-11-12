using System;
using UnityEngine;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.PathFinding;

namespace vFrame.Lockstep.Core.Editor
{
    [Serializable]
    public class NavMeshTool : MonoBehaviour
    {
        public int mapId;
        public GameObject mapRoot;
        public TriangleData data = new TriangleData();
    }
}