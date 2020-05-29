using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace vFrame.Lockstep.Core.PathFinding.Grid
{
    public class GridGraph : IIndexedGraph<GridNode>
    {
        private readonly GridData _data;
        private readonly Dictionary<int, Dictionary<int, List<IConnection<GridNode>>>> _connections;
        private readonly Dictionary<int, Dictionary<int, GridNode>> _nodes;
        private readonly int _mask;

        public GridGraph(GridData data, int mask) {
            _data = data;
            _nodes = new Dictionary<int, Dictionary<int, GridNode>>(_data.cols);
            _connections = new Dictionary<int, Dictionary<int, List<IConnection<GridNode>>>>(_data.cols);
            _mask = mask;

            BuildGraph();
        }

        public List<IConnection<GridNode>> GetConnections(GridNode fromNode) {
            return _connections[fromNode.xIndex][fromNode.yIndex];
        }

        public GridNode GetNode(int x, int y) {
            Debug.Assert(x >=0 && x < _data.cols);
            Debug.Assert(y >=0 && y < _data.rows);
            return _nodes[x][y];
        }

        public GridNode GetNode(TSVector2 pos) {
            var x = TSMath.Floor((pos.x - _data.start.x) / _data.width);
            var y = TSMath.Floor((pos.y - _data.start.y) / _data.width);
            if (x < 0 || x >= _data.cols || y < 0 || y >= _data.rows ) {
                throw new IndexOutOfRangeException($"pos: {pos}, graph start: {_data.start}, width: {_data.width}, "
                    + $"rows: {_data.rows}, cols: {_data.cols}");
            }
            return _nodes[(int)x][(int)y];
        }

        public int GetIndex(GridNode node) {
            return node.index;
        }

        public int GetNodeCount() {
            return _data.grids.Length;
        }

        private void BuildGraph() {
            Debug.Assert(_data.grids.Length == _data.rows * _data.cols);

            // 构建所有结点
            for (var x = 0; x < _data.cols; x++) {
                _nodes.Add(x, new Dictionary<int, GridNode>(_data.rows));
                for (var y = 0; y < _data.rows; y++) {
                    var index = x * _data.cols + y;
                    var node = new GridNode {
                        flag = _data.grids[index],
                        index = index,
                        xIndex = x,
                        yIndex = y,
                        center = _data.start + new TSVector2(
                                     ((FixedPoint) x + 0.5f) * _data.width,
                                     ((FixedPoint) y + 0.5f) * _data.width)
                    };
                    _nodes[x].Add(y, node);
                }
            }

            // 构建所有连接
            for (var x = 0; x < _data.cols; x++) {
                _connections.Add(x, new Dictionary<int, List<IConnection<GridNode>>>(_data.rows));
                for (var y = 0; y < _data.rows; y++) {
                    var curNode = _nodes[x][y];
                    var connections = new List<IConnection<GridNode>>();
                    // 遍历上下左右节点，构造连接
                    if (y < _data.rows - 1 && (_nodes[x][y + 1].flag & _mask) > 0) {
                        connections.Add(new GridConnection(curNode, _nodes[x][y+1])); // 上
                    }
                    if (y > 0 && (_nodes[x][y - 1].flag & _mask) > 0) {
                        connections.Add(new GridConnection(curNode, _nodes[x][y-1])); // 下
                    }
                    if (x > 0 && (_nodes[x-1][y].flag & _mask) > 0) {
                        connections.Add(new GridConnection(curNode, _nodes[x-1][y])); // 左
                    }
                    if (x < _data.cols - 1 && (_nodes[x+1][y].flag & _mask) > 0) {
                        connections.Add(new GridConnection(curNode, _nodes[x+1][y])); // 右
                    }
                    _connections[x].Add(y, connections);
                }
            }
        }
    }
}