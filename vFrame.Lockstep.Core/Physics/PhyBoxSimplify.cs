using System.Collections.Generic;
using System.Diagnostics;
using System.Xml;

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// 封装边的逻辑
    /// </summary>
    public class MergeShape
    {
        private Transform m_xf; // the body origin transform
        public TSVector2 m_center;
        public TSVector2 m_forward;

        public int m_layer;
        public string m_name;
        public uint m_source;
        public Vertices m_vertex = new Vertices();
        public bool m_loop = true;
        public bool m_canMerge = true;

        public int EdgeCount
        {
            get { return m_loop ? m_vertex.Count : m_vertex.Count - 1; }
        }

        public void GetEdge(int index, out TSVector2 start, out TSVector2 end)
        {
            start = GetWorldPoint(m_vertex[index]);
            end = GetWorldPoint(m_vertex[m_vertex.NextIndex(index)]);
        }

        public TSVector2 GetHeadWorldPos()
        {
            return GetWorldPoint(m_vertex[0]);
        }

        public TSVector2 GetTailWorldPos()
        {
            return GetWorldPoint(m_vertex[m_vertex.Count - 1]);
        }

        public TSVector2 GetVertextPos(int index)
        {
            return GetWorldPoint(m_vertex[index]);
        }

        public TSVector2 GetWorldPoint(TSVector2 localPoint)
        {
            return MathUtils.Mul(ref m_xf, ref localPoint);
        }
        public TSVector2 GetLocalPoint(TSVector2 worldPoint)
        {
            return MathUtils.MulT(ref m_xf, worldPoint);
        }

        public MergeShape(string name, int layer, uint source, TSVector2 center, TSVector2 forward, Vertices vertex, bool loop, bool canMerge = true)
        {
            m_center = center;
            m_forward = forward;

            m_name = name;
            m_layer = layer;
            m_source = source;
            m_xf.Set(center, forward);
            m_vertex.AddRange(vertex);
            m_loop = loop;
            m_canMerge = canMerge;
        }

        public bool IsPointMatch(TSVector2 p1, TSVector2 p2)
        {
            var distSqrt = TSVector2.DistanceSquared(p1, p2);
            return distSqrt <= FixedPoint.EN4;
        }

        public int FindVertexIndex(TSVector2 worldVert)
        {
            var localVert = GetLocalPoint(worldVert);
            for (int i = 0; i < m_vertex.Count; i++)
            {
                if (IsPointMatch(m_vertex[i], localVert))
                {
                    return i;
                }
            }

            return -1;
        }

        public bool TryReplaceVertex(int index, TSVector2 worldPos)
        {
            var localVert = GetLocalPoint(worldPos);

            var current = m_vertex[index];
            var prev = localVert;
            var next = m_vertex[1 - index];

            ///要判断是否在一条线上
            if (MathUtils.IsCollinear(ref prev, ref current, ref next, FixedPoint.Zero))
            {
                m_vertex[index] = localVert;
                return true;
            }

            return false;
        }

        public int FindEdge(TSVector2 start, TSVector2 end)
        {
            for (int i = 0; i < EdgeCount; i++)
            {
                TSVector2 edgeStart;
                TSVector2 edgeEnd;

                GetEdge(i, out edgeStart, out edgeEnd);

                if (IsPointMatch(edgeStart, start) && IsPointMatch(edgeEnd, end) ||
                    IsPointMatch(edgeStart, end) && IsPointMatch(edgeEnd, start))
                {
                    return i;
                }
            }

            return -1;
        }

        public List<MergeShape> CutEdge(int index, FixedPoint cutPoint, bool leftOrRight)
        {
            ///先彻底切断，然后再拼一个进去
            TSVector2 cutStart, cutEnd;
            var listMerge = CutEdge(index, out cutStart, out cutEnd);
            var newVert = new Vertices(2);
            if (leftOrRight)
            {
                newVert.Add(cutStart);
                newVert.Add(TSVector2.Lerp(cutStart, cutEnd, cutPoint));
            }
            else
            {
                newVert.Add(TSVector2.Lerp(cutStart, cutEnd, cutPoint));
                newVert.Add(cutEnd);
            }

            listMerge.Add(CreateChildShape(newVert));
            return listMerge;
        }

        public List<MergeShape> CutEdge(int index)
        {
            TSVector2 cutStart;
            TSVector2 cutEnd;
            return CutEdge(index, out cutStart, out cutEnd);
        }

        public MergeShape FetchEdge(int index)
        {
            Vertices vertex = new Vertices();
            vertex.Add(m_vertex[index]);
            if (m_loop)
            {
                vertex.Add(m_vertex[m_vertex.NextIndex(index+1)]);
            }
            else
            {
                vertex.Add(m_vertex[index+1]);
            }

            return CreateChildShape(vertex);
        }

        private MergeShape CreateChildShape(Vertices vertex)
        {
            return new MergeShape(m_name, m_layer, m_source, m_center, m_forward, vertex, false);
        }

        private List<MergeShape> CutEdge(int index, out TSVector2 cutStart, out TSVector2 cutEnd)
        {
            var listMerge = new List<MergeShape>();

            if (m_loop)
            {
                cutStart = m_vertex[index];
                cutEnd = m_vertex[m_vertex.NextIndex(index)];

                var newVert = new Vertices();
                var currIndex = m_vertex.NextIndex(index);
                while (true)
                {
                    newVert.Add(m_vertex[currIndex]);
                    if (currIndex == index)
                    {
                        break;
                    }

                    currIndex = m_vertex.NextIndex(currIndex);
                }

                var shape = CreateChildShape(newVert);
                listMerge.Add(shape);
            }
            else
            {
                cutStart = m_vertex[index];
                cutEnd = m_vertex[index + 1];

                var newVert = new Vertices();
                for (int i = 0; i <= index; i++)
                {
                    newVert.Add(m_vertex[i]);
                }

                if (newVert.Count > 1)
                {
                    var shape = CreateChildShape(newVert);
                    listMerge.Add(shape);
                }

                newVert = new Vertices();
                for (int i = index+1; i < m_vertex.Count; i++)
                {
                    newVert.Add(m_vertex[i]);
                }

                if (newVert.Count > 1)
                {
                    var shape = CreateChildShape(newVert);
                    listMerge.Add(shape);
                }
            }

            return listMerge;
        }


        #region 合并到chain

        public bool TryMergeChain(MergeShape shape)
        {
            Debug.Assert(!m_loop);

            var head = shape.GetHeadWorldPos();
            var tail = shape.GetTailWorldPos();

            head = GetLocalPoint(head);
            tail = GetLocalPoint(tail);

            var myHead = m_vertex[0];
            var myTail = m_vertex[m_vertex.Count - 1];
            if (IsPointMatch(head, myHead))
            {
                for (int i = 1; i < shape.m_vertex.Count; i++)
                {
                    AddToHeadWorldPos(shape.GetVertextPos(i));
                }

                return true;
            }
            else if (IsPointMatch(tail, myHead))
            {
                for (int i = shape.m_vertex.Count - 2; i >= 0; i--)
                {
                    AddToHeadWorldPos(shape.GetVertextPos(i));
                }

                return true;
            }
            else if (IsPointMatch(head, myTail))
            {
                for (int i = 1; i < shape.m_vertex.Count; i++)
                {
                    AddToTailWorldPos(shape.GetVertextPos(i));
                }

                return true;
            }
            else if (IsPointMatch(tail, myTail))
            {
                for (int i = shape.m_vertex.Count - 2; i >= 0; i--)
                {
                    AddToTailWorldPos(shape.GetVertextPos(i));
                }

                return true;
            }

            return false;
        }

        private void AddToHeadWorldPos(TSVector2 worldPos)
        {
            var pos = GetLocalPoint(worldPos);
            m_vertex.Insert(0, pos);
        }
        private void AddToTailWorldPos(TSVector2 worldPos)
        {
            var pos = GetLocalPoint(worldPos);
            m_vertex.Add(pos);
        }
        #endregion
    }

    /// <summary>
    /// 进行box阻挡的合并操作
    /// </summary>
    public class PhyBoxSimplify
    {
        private List<MergeShape> m_listShape = new List<MergeShape>();

        private uint m_nextSource = 1;
        public void AddBox(string name, TSVector2 center, TSVector2 forward, TSVector2 size, int layer, bool canMerge = true)
        {
            var extend = size * FixedPoint.Half;
            var shape = new MergeShape(name, layer, m_nextSource++, center, forward, PolygonTools.CreateRectangle(extend.x, extend.y), true, canMerge);
            m_listShape.Add(shape);
        }

        /// <summary>
        /// 简化过后的形状和顶点
        /// </summary>
        /// <returns></returns>
        public List<MergeShape>  Simplify()
        {
            var listMerge = new List<MergeShape>();
            for (int i = 0; i < m_listShape.Count; i++)
            {
                SimplifyShape(m_listShape[i], m_listShape, listMerge);
            }

            List<MergeShape> listEdage = new List<MergeShape>();
            SplitToEdage(listMerge, listEdage);
            return listEdage;
        }

        void SplitToEdage(List<MergeShape> listSource, List<MergeShape> listFinal)
        {
            List<MergeShape> listEdge =new List<MergeShape>();
            for (int i = 0; i < listSource.Count; i++)
            {
                var source = listSource[i];
                if (source.m_loop)
                {
                    listFinal.Add(source);
                    continue;
                }

                for (int k = 0; k < source.EdgeCount; k++)
                {
                    listEdge.Add(source.FetchEdge(k));
                }
            }

            List<MergeShape> listBigEdge = new List<MergeShape>();
            MergeEdage(listEdge, listBigEdge);


            listFinal.AddRange(listBigEdge);
           // MergeEdageToPolygon(listBigEdge, listFinal);
            //RecalSource(listFinal);
        }

        void RecalSource(List<MergeShape> listShape)
        {
            for (int i = 0; i < listShape.Count; i++)
            {
                var shape = listShape[i];
                shape.m_source = (uint) (i + 1);
            }
        }

        void MergeEdageToChain(List<MergeShape> listSource, List<MergeShape> listChain)
        {
            for (int i = 0; i < listSource.Count; i++)
            {
                var currShape = listSource[i];
                var mergedToNext = false;
                for (int k = i + 1; k < listSource.Count; k++)
                {
                    var shape2 = listSource[k];
                    if (TryMergeChain(currShape, shape2))
                    {
                        mergedToNext = true;
                        break;
                    }
                }

                if (!mergedToNext)
                {
                    listChain.Add(currShape);
                }
            }
        }

        /// <summary>
        /// 把可以合并成正方形的合并一下
        /// </summary>
        /// <param name="listSource"></param>
        /// <param name="listFinal"></param>
        void MergeEdageToPolygon(List<MergeShape> listSource, List<MergeShape> listFinal)
        {
            ///先尽量合并，然后判断是否转换为polygon
            List<MergeShape> listChain = new List<MergeShape>();
            MergeEdageToChain(listSource, listChain);

            for (int i = 0; i < listChain.Count; i++)
            {
                var shape = listChain[i];
                if (shape.EdgeCount == 1)
                {
                    listFinal.Add(shape);
                }
                else if (shape.EdgeCount != 4)
                {
                    for (int k = 0; k < shape.EdgeCount; k++)
                    {
                        listFinal.Add(shape.FetchEdge(k));
                    }
                }
                else
                {
                    Debug.Assert(shape.EdgeCount == 4);
                    Debug.Assert(shape.m_vertex.Count == 5);
                    Debug.Assert(!shape.m_loop);

                    //判断是否是polygon
                    if (shape.IsPointMatch(shape.m_vertex[0], shape.m_vertex[shape.m_vertex.Count - 1]))
                    {
                        shape.m_vertex.RemoveAt(shape.m_vertex.Count-1);
                        shape.m_loop = true;

                        listFinal.Add(shape);
                    }
                    else
                    {
                        for (int k = 0; k < shape.EdgeCount; k++)
                        {
                            listFinal.Add(shape.FetchEdge(k));
                        }
                    }
                }
            }
        }

        void MergeEdage(List<MergeShape> listSource, List<MergeShape> listFinal)
        {
            for (int i = 0; i < listSource.Count; i++)
            {
                var currShape = listSource[i];
                var mergedToNext = false;
                for (int k = i + 1; k < listSource.Count; k++)
                {
                    var shape2 = listSource[k];
                    if (TryMerge(currShape, shape2))
                    {
                        mergedToNext = true;
                        break;
                    }
                }

                if (!mergedToNext)
                {
                    listFinal.Add(currShape);
                }
            }
        }

        bool TryMergeChain(MergeShape shape1, MergeShape shape2)
        {
            return shape2.TryMergeChain(shape1);
        }

        bool TryMerge(MergeShape shape1, MergeShape shape2)
        {
            if (shape1.m_layer != shape2.m_layer)
            {
                return false;
            }

            Debug.Assert(shape1.EdgeCount == 1 && shape2.EdgeCount == 1);

            TSVector2 start1, end1;
            shape1.GetEdge(0, out start1, out end1);

            var index = shape2.FindVertexIndex(start1);
            if (index >= 0)
            {
                if (shape2.TryReplaceVertex(index, end1))
                {
                    return true;
                }
            }

            index = shape2.FindVertexIndex(end1);
            if (index >= 0)
            {
                if (shape2.TryReplaceVertex(index, start1))
                {
                    return true;
                }
            }

            return false;
        }

        void SimplifyShape(MergeShape shape, List<MergeShape> sourceList, List<MergeShape> resShapeList)
        {
            if (!shape.m_canMerge)
            {
                resShapeList.Add(shape);
                return;
            }

            List<MergeShape> toCheckList = new List<MergeShape>();
            toCheckList.Add(shape);

            for (int i = 0; i < toCheckList.Count; i++)
            {
                var toCheckShaper = toCheckList[i];

                List<MergeShape> listSubShaper;
                if (CheckCut(toCheckShaper, sourceList, out listSubShaper))
                {
                    toCheckList.AddRange(listSubShaper);
                }
                else
                {
                    resShapeList.Add(toCheckShaper);
                }
            }
        }

        bool CheckCut(MergeShape shape, List<MergeShape> sourceList, out List<MergeShape> cutAfterShaper)
        {
            cutAfterShaper = null;

            for (int i = 0; i < sourceList.Count; i++)
            {
                var source = sourceList[i];
                if (shape.m_source == source.m_source ||
                    shape.m_layer != source.m_layer)
                {
                    continue;
                }

                int cutIndex;
                if (CheckShapeSameEdage(shape, source, out cutIndex))
                {
                    cutAfterShaper = shape.CutEdge(cutIndex);
                    return true;
                }
            }

            return false;
        }

        private bool CheckShapeSameEdage(MergeShape checkShape, MergeShape sourceShape, out int cutIndex)
        {
            for (int i = 0; i < checkShape.EdgeCount; i++)
            {
                TSVector2 checkStart, checkEnd;
                checkShape.GetEdge(i, out checkStart, out checkEnd);

                int sourceIndex = sourceShape.FindEdge(checkStart, checkEnd);
                if (sourceIndex >= 0)
                {
                    cutIndex = i;
                    return true;
                }
            }

            cutIndex = -1;
            return false;
        }
    }
}