namespace vFrame.Lockstep.Core.Physics2D {

    internal class ResourcePoolBodyClone2D : ResourcePool<BodyClone2D> {

        protected override BodyClone2D NewInstance() {
            return new BodyClone2D();
        }

    }
    
    internal class ResourcePoolTreeFixtureProxy2D : ResourcePool<TreeNode<FixtureProxy>> {

        protected override TreeNode<FixtureProxy> NewInstance() {
            return new TreeNode<FixtureProxy>();
        }

    }

    internal class ResourcePoolShapeClone2D : ResourcePool<GenericShapeClone2D> {

        protected override GenericShapeClone2D NewInstance() {
            return new GenericShapeClone2D();
        }

    }

}