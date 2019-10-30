using System.Collections.Generic;

namespace vFrame.Lockstep.Core.Physics2D
{
    internal class ContactCloneKeyComparer : IEqualityComparer<ContactCloneKey>
    {
        public bool Equals(ContactCloneKey x, ContactCloneKey y) {
            return x.keyA == y.keyA && x.keyB == y.keyB;
        }

        public int GetHashCode(ContactCloneKey obj) {
            return obj.keyA.GetHashCode() * 33 + obj.keyB.GetHashCode() * 17;
        }
    }

    internal class ContactEdgeKeyComparer : IEqualityComparer<ContactEdgeCloneKey>
    {
        public bool Equals(ContactEdgeCloneKey x, ContactEdgeCloneKey y) {
            return x.contactKey.Equals(y.contactKey) && x.bodyId == y.bodyId;
        }

        public int GetHashCode(ContactEdgeCloneKey obj) {
            return obj.contactKey.GetHashCode() * 33 + obj.bodyId.GetHashCode();
        }
    }

    internal struct ContactCloneKey
    {
        internal static ContactCloneKey empty = new ContactCloneKey(-1, -1);

        public int keyA;

        public int keyB;

        public ContactCloneKey(int keyA, int keyB) {
            this.keyA = keyA;
            this.keyB = keyB;
        }
    }

    internal struct ContactEdgeCloneKey
    {
        public ContactCloneKey contactKey;

        public int bodyId;

        public ContactEdgeCloneKey(ContactCloneKey contactKey, int bodyId) {
            this.contactKey = contactKey;
            this.bodyId = bodyId;
        }

        public void Set(ContactCloneKey contactKey, int bodyId) {
            this.contactKey = contactKey;
            this.bodyId = bodyId;
        }
    }

    public class WorldClone2D : IWorldClone
    {
        internal static ResourcePoolBodyClone2D poolRigidBodyClone = new ResourcePoolBodyClone2D();
        internal static ResourcePoolTreeFixtureProxy2D poolTreeFixtureProxy = new ResourcePoolTreeFixtureProxy2D();

        public string checksum { get; private set; }

        internal int bodyCounter;

        internal int fixtureCounter;

        internal Dictionary<int, BodyClone2D> clonedPhysics = new Dictionary<int, BodyClone2D>();

        internal bool _worldHasNewFixture;
        internal TOIClone2D toiClone = new TOIClone2D();

        internal DynamicTreeBroadPhaseClone2D dynamicTreeClone = new DynamicTreeBroadPhaseClone2D();
        private List<Physics2D.Body> bodiesToRemove = new List<Physics2D.Body>();

        private int index, length;

        public void Reset() {
            var clonedPhysicsEnum = clonedPhysics.GetEnumerator();
            while (clonedPhysicsEnum.MoveNext()) {
                BodyClone2D cl = clonedPhysicsEnum.Current.Value;

                cl.Reset();
                poolRigidBodyClone.GiveBack(cl);
            }
        }

        public void Clone(IWorld iWorld) {
            Clone(iWorld, false);
        }

        public void Clone(IWorld iWorld, bool doChecksum) {
            Physics2D.World world = (Physics2D.World) iWorld;

            Reset();

            if (doChecksum) {
                checksum = ChecksumExtractor.GetEncodedChecksum();
            }

            clonedPhysics.Clear();
            for (index = 0, length = world.BodyList.Count; index < length; index++) {
                Physics2D.Body b = world.BodyList[index];

                BodyClone2D cloneB = poolRigidBodyClone.GetNew();
                cloneB.Clone(b);

                clonedPhysics.Add(b.BodyId, cloneB);
            }

            toiClone.Clone(world._input);
            dynamicTreeClone.Clone((Physics2D.DynamicTreeBroadPhase) world.BroadPhase);

            this._worldHasNewFixture = world._worldHasNewFixture;

            bodyCounter = Physics2D.Body._bodyIdCounter;
            fixtureCounter = Physics2D.Fixture._fixtureIdCounter;
        }

        public void Restore(IWorld iWorld) {
            Physics2D.World world = (Physics2D.World) iWorld;

            bodiesToRemove.Clear();
            for (index = 0, length = world.BodyList.Count; index < length; index++) {
                Physics2D.Body rb = world.BodyList[index];

                if (!clonedPhysics.ContainsKey(rb.BodyId)) {
                    bodiesToRemove.Add(rb);
                }
            }

            for (index = 0, length = bodiesToRemove.Count; index < length; index++) {
                Physics2D.Body rb = bodiesToRemove[index];

                world.RemoveBody(rb);
            }

            world.ProcessRemovedBodies();

            for (index = 0, length = world.BodyList.Count; index < length; index++) {
                Physics2D.Body rb = world.BodyList[index];

                if (clonedPhysics.ContainsKey(rb.BodyId)) {
                    BodyClone2D rbClone = clonedPhysics[rb.BodyId];
                    rbClone.Restore(rb);
                }
            }

            toiClone.Restore(world._input);

            TreeNode<FixtureProxy>[] treeNodes = ((DynamicTreeBroadPhase) world.BroadPhase)._tree._nodes;
            for (index = 0, length = treeNodes.Length; index < length; index++) {
                TreeNode<FixtureProxy> tn = treeNodes[index];

                poolTreeFixtureProxy.GiveBack(tn);
            }

            dynamicTreeClone.Restore((DynamicTreeBroadPhase) world.BroadPhase);
            world._worldHasNewFixture = this._worldHasNewFixture;

            Body._bodyIdCounter = bodyCounter;
            Fixture._fixtureIdCounter = fixtureCounter;
        }
    }
}