namespace vFrame.Lockstep.Core.PathFinding;

public class IndexedAStarPathFinder<TNode> : IPathFinder<TNode>
{
    private const int UNVISITED = 0;
    private const int OPEN = 1;
    private const int CLOSED = 2;
    private readonly IIndexedGraph<TNode> _graph;
    private readonly Metrics _metrics;
    private readonly NodeRecord<TNode>[] _nodeRecords;

    private readonly NodeBinaryHeap<NodeRecord<TNode>> _openList;
    private NodeRecord<TNode> _current;

    /**
     * The unique ID for each search run. Used to mark nodes.
     */
    private int _searchId;

    public IndexedAStarPathFinder(IIndexedGraph<TNode> graph) : this(graph, false) {
    }

    public IndexedAStarPathFinder(IIndexedGraph<TNode> graph, bool calculateMetrics) {
        _graph = graph;
        _nodeRecords = new NodeRecord<TNode>[graph.GetNodeCount()];
        _openList = new NodeBinaryHeap<NodeRecord<TNode>>();
        if (calculateMetrics)
            _metrics = new Metrics();
    }

    public bool SearchPath(TNode startNode, TNode endNode, IHeuristic<TNode> heuristic,
        IGraphPath<IConnection<TNode>> outPath) {
        if (startNode == null)
            return false;

        if (endNode == null)
            return false;

        // Perform AStar
        var found = Search(startNode, endNode, heuristic);

        if (found) // Create a path made of connections
            GeneratePath(startNode, outPath);

        return found;
    }

    public bool SearchNodePath(TNode startNode, TNode endNode, IHeuristic<TNode> heuristic, IGraphPath<TNode> outPath) {
        // Perform AStar
        var found = Search(startNode, endNode, heuristic);

        if (found) // Create a path made of nodes
            GenerateNodePath(startNode, outPath);

        return found;
    }

    protected bool Search(TNode startNode, TNode endNode, IHeuristic<TNode> heuristic) {
        InitSearch(startNode, endNode, heuristic);

        // Iterate through processing each node
        do {
            // Retrieve the node with smallest estimated total cost from the open list
            _current = _openList.Pop();
            _current.category = CLOSED;

            // Terminate if we reached the goal node
            if (_current.node.Equals(endNode))
                return true;

            VisitChildren(endNode, heuristic);
        }
        while (_openList.Size > 0);

        // We've run out of nodes without finding the goal, so there's no solution
        return false;
    }

    protected void InitSearch(TNode startNode, TNode endNode, IHeuristic<TNode> heuristic) {
        _metrics?.Reset();

        // Increment the search id
        if (++_searchId < 0)
            _searchId = 1;

        // Initialize the open list
        _openList.Clear();

        // Initialize the record for the start node and add it to the open list
        var startRecord = GetNodeRecord(startNode);
        startRecord.node = startNode;
        startRecord.connection = null;
        startRecord.costSoFar = new FixedPoint(0);
        AddToOpenList(startRecord, heuristic.Estimate(startNode, endNode));

        _current = null;
    }

    protected void VisitChildren(TNode endNode, IHeuristic<TNode> heuristic) {
        // Get current node's outgoing connections
        var connections = _graph.GetConnections(_current.node);

        // Loop through each connection in turn
        for (var i = 0; i < connections.Count; i++) {
            if (_metrics != null)
                _metrics.VisitedNodes++;

            var connection = connections[i];

            // Get the cost estimate for the node
            var node = connection.GetToNode(); //周围目标节点
            var nodeCost = _current.costSoFar + connection.GetCost(); //节点到目标的消耗

            FixedPoint nodeHeuristic;
            var nodeRecord = GetNodeRecord(node);
            if (nodeRecord.category == CLOSED) { // The node is closed

                // If we didn't find a shorter route, skip 已经是消耗最小的目标点
                if (nodeRecord.costSoFar <= nodeCost)
                    continue;

                // We can use the node's old cost values to calculate its heuristic
                // without calling the possibly expensive heuristic function
                nodeHeuristic = nodeRecord.GetEstimatedTotalCost() - nodeRecord.costSoFar;
            }
            else if (nodeRecord.category == OPEN) { // The node is open

                // If our route is no better, then skip
                if (nodeRecord.costSoFar <= nodeCost)
                    continue;

                // Remove it from the open list (it will be re-added with the new cost)
                _openList.Remove(nodeRecord);

                // We can use the node's old cost values to calculate its heuristic
                // without calling the possibly expensive heuristic function
                nodeHeuristic = nodeRecord.GetEstimatedTotalCost() - nodeRecord.costSoFar;
            }
            else { // the node is unvisited

                // We'll need to calculate the heuristic value using the function,
                // since we don't have a node record with a previously calculated value
                nodeHeuristic = heuristic.Estimate(node, endNode);
            }

            // Update node record's cost and connection
            nodeRecord.costSoFar = nodeCost;
            nodeRecord.connection = connection;

            // Add it to the open list with the estimated total cost
            AddToOpenList(nodeRecord, nodeCost + nodeHeuristic);
        }
    }

    protected void GeneratePath(TNode startNode, IGraphPath<IConnection<TNode>> outPath) {
        // Work back along the path, accumulating connections
        // outPath.clear();
        while (!_current.node.Equals(startNode)) {
            outPath.Add(_current.connection);
            _current = _nodeRecords[_graph.GetIndex(_current.connection.GetFromNode())];
        }

        // Reverse the path
        outPath.Reverse();
    }

    protected void GenerateNodePath(TNode startNode, IGraphPath<TNode> outPath) {
        // Work back along the path, accumulating nodes
        // outPath.clear();
        while (_current.connection != null) {
            outPath.Add(_current.node);
            _current = _nodeRecords[_graph.GetIndex(_current.connection.GetFromNode())];
        }

        outPath.Add(startNode);

        // Reverse the path
        outPath.Reverse();
    }

    protected void AddToOpenList(NodeRecord<TNode> nodeRecord, FixedPoint estimatedTotalCost) {
        _openList.Add(nodeRecord, estimatedTotalCost);
        nodeRecord.category = OPEN;
        if (_metrics != null) {
            ++_metrics.OpenListAdditions;
            if (_openList.Size > _metrics.OpenListPeak)
                _metrics.OpenListPeak = _openList.Size;
        }
    }

    protected NodeRecord<TNode> GetNodeRecord(TNode node) {
        var index = _graph.GetIndex(node);
        var nr = _nodeRecords[index];
        if (nr != null) {
            if (nr.searchId != _searchId) {
                nr.category = UNVISITED;
                nr.searchId = _searchId;
            }

            return nr;
        }

        nr = _nodeRecords[index] = new NodeRecord<TNode>();
        nr.node = node;
        nr.searchId = _searchId;
        return nr;
    }

    public class NodeRecord<T> : Node
    {
        public int category;

        public IConnection<T> connection;

        public FixedPoint costSoFar;
        public T node;

        public int searchId;

        public NodeRecord() : base(FixedPoint.Zero) {
        }

        public FixedPoint GetEstimatedTotalCost() {
            return value;
        }
    }

    public class Metrics
    {
        public int OpenListAdditions;
        public int OpenListPeak;
        public int VisitedNodes;

        public void Reset() {
            VisitedNodes = 0;
            OpenListAdditions = 0;
            OpenListPeak = 0;
        }
    }
}