using System;
using System.Text;

namespace vFrame.Lockstep.Core.PathFinding
{
    public class Node
    {
        public FixedPoint value; //节点排序比较值
        public int index; //节点索引

        public Node(FixedPoint value) {
            this.value = value;
        }

        public FixedPoint GetValue() {
            return value;
        }

        public override string ToString() {
            return value.ToString();
        }
    }

    public class NodeBinaryHeap<T> where T : Node
    {
        public int size;

        private Node[] _nodes;
        private bool _isMaxHeap;

        public NodeBinaryHeap() : this(16, false) {
        }

        public NodeBinaryHeap(int capacity, bool isMaxHeap) {
            _isMaxHeap = isMaxHeap;
            _nodes = new Node[capacity];
        }

        public T Add(T node) {
            // Expand if necessary.
            if (size == _nodes.Length) {
                var newNodes = new Node[size << 1];
                Array.Copy(_nodes, 0, newNodes, 0, size);
                _nodes = newNodes;
            }

            // Insert at end and bubble up.
            node.index = size;
            _nodes[size] = node;
            Up(size++);
            return node;
        }

        public T Add(T node, FixedPoint value) {
            node.value = value;
            return Add(node);
        }

        public T Peek() {
            if (size == 0)
                throw new Exception("The heap is empty.");
            return (T) _nodes[0];
        }

        public T Pop() {
            return Remove(0);
        }

        public T Remove(T node) {
            return Remove(node.index);
        }

        private T Remove(int index) {
            var nodes = _nodes;
            var removed = nodes[index];
            nodes[index] = nodes[--size];
            nodes[size] = null;
            if (size > 0 && index < size)
                Down(index);
            return (T) removed;
        }

        //TODO 可以不用对nodes 进行清零
        public void Clear() {
            var nodes = _nodes;
            for (int i = 0, n = size; i < n; i++)
                nodes[i] = null;
            size = 0;
        }

        public void SetValue(T node, FixedPoint value) {
            var oldValue = node.value;
            node.value = value;
            if ((value < oldValue) ^ _isMaxHeap)
                Up(node.index);
            else
                Down(node.index);
        }

        private void Up(int index) {
            var nodes = _nodes;
            var node = nodes[index];
            var value = node.value;
            while (index > 0) {
                var parentIndex = (index - 1) >> 1;
                var parent = nodes[parentIndex];
                if ((value < parent.value) ^ _isMaxHeap) {
                    nodes[index] = parent;
                    parent.index = index;
                    index = parentIndex;
                }
                else {
                    break;
                }
            }

            nodes[index] = node;
            node.index = index;
        }

        public static readonly FixedPoint MinValue = new FixedPoint(int.MinValue);

        private void Down(int index) {
            var nodes = _nodes;
            var size = this.size;

            var node = nodes[index];
            var value = node.value;

            while (true) {
                var leftIndex = 1 + (index << 1);
                if (leftIndex >= size)
                    break;

                // Always have a left child.
                var leftNode = nodes[leftIndex];
                var leftValue = leftNode.value;

                // May have a right child.
                var rightIndex = leftIndex + 1;
                Node rightNode;
                FixedPoint rightValue;
                if (rightIndex >= size) {
                    rightNode = null;
                    rightValue = _isMaxHeap ? FixedPoint.MinValue : FixedPoint.MaxValue;
                }
                else {
                    rightNode = nodes[rightIndex];
                    rightValue = rightNode.value;
                }

                // The smallest of the three values is the parent.
                if ((leftValue < rightValue) ^ _isMaxHeap) {
                    if (leftValue == value || (leftValue > value) ^ _isMaxHeap)
                        break;
                    nodes[index] = leftNode;
                    leftNode.index = index;
                    index = leftIndex;
                }
                else {
                    if (rightValue == value || (rightValue > value) ^ _isMaxHeap)
                        break;
                    nodes[index] = rightNode;
                    rightNode.index = index;
                    index = rightIndex;
                }
            }

            nodes[index] = node;
            node.index = index;
        }


        public override int GetHashCode() {
            var h = 1;
            for (int i = 0, n = size; i < n; i++)
                h = h * 31 + _nodes[i].value.AsInt() * 1000;
            return h;
        }


        public override string ToString() {
            if (size == 0)
                return "[]";
            var nodes = _nodes;
            var buffer = new StringBuilder(32);
            buffer.Append('[');
            buffer.Append(nodes[0].value);
            for (var i = 1; i < size; i++) {
                buffer.Append(", ");
                buffer.Append(nodes[i].value);
            }

            buffer.Append(']');
            return buffer.ToString();
        }
    }
}