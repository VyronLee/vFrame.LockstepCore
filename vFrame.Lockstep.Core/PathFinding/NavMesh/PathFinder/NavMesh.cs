namespace vFrame.Lockstep.Core.PathFinding
{
    public abstract class NavMesh
    {
        /** 地图宽x轴 */
        protected FixedPoint width;

        /** 地图高y轴 */
        protected FixedPoint height;

        /** 配置id */
        protected int mapId;

        public FixedPoint getWidth() {
            return width;
        }

        public void setWidth(FixedPoint width) {
            this.width = width;
        }

        public FixedPoint getHeight() {
            return height;
        }

        public void setHeight(FixedPoint height) {
            this.height = height;
        }

        public int getMapId() {
            return mapId;
        }

        public void setMapId(int mapId) {
            this.mapId = mapId;
        }
    }
}