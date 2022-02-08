namespace vFrame.Lockstep.Core.PathFinding.Grid;

public class GridNode
{
    /// <summary>
    ///     格子中心坐标
    /// </summary>
    public TSVector2 center;

    /// <summary>
    ///     格子标记
    /// </summary>
    public int flag;

    /// <summary>
    ///     格子序号
    /// </summary>
    public int index;

    /// <summary>
    ///     列序号
    /// </summary>
    public int xIndex;

    /// <summary>
    ///     行序号
    /// </summary>
    public int yIndex;
}