using vFrame.Lockstep.Core.Physics2D;

namespace vFrame.Lockstep.Core
{
    /// <summary>
    /// 重新抽象一个GameObject
    /// </summary>
    public class PhyGameObject
    {
        public string name;
        public int layer;
        public Body body;
        
        public PhyGameObject(string _name)
        {
            name = _name;
        }

        public TSVector2 Position
        {
            set { body.Position = value; }
            get { return body.Position; }
        }

        public TSVector2 Forward
        {
            set { body.Forward = value; }
            get { return body.Forward; }
        }

        public void SetTransform(TSVector2 pos, TSVector2 forward)
        {
            body.SetTransform(ref pos, ref forward);
        }
    }
}