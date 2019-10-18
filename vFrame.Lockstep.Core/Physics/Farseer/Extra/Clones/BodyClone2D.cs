using System.Collections.Generic;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.Physics2D;

namespace vFrame.Lockstep.Core.Physics2D {

    internal class BodyClone2D {

        private static ResourcePoolShapeClone2D poolClone2D = new ResourcePoolShapeClone2D();

        public BodyType _bodyType;

        public bool _sleepingAllowed = true;
        public bool _awake = true;

        public bool _enabled = true;
        public FixedPoint _sleepTime;

        public Transform _xf = new Transform();
        public bool disabled;

        public List<GenericShapeClone2D> shapesClone = new List<GenericShapeClone2D>();

        public void Reset() {
        }

        public void Clone(Body body) {
            this._bodyType = body._bodyType;
            this._sleepingAllowed = body._sleepingAllowed;
            this._awake = body._awake;
            this._enabled = body._enabled;
            this._sleepTime = body._sleepTime;
            
            this._xf.p = body._xf.p;
            this._xf.q = body._xf.q;

            this.disabled = body.disabled;
            
            for (int index = 0, length = shapesClone.Count; index < length; index++) {
                poolClone2D.GiveBack(shapesClone[index]);
            }

            this.shapesClone.Clear();

            List<Physics2D.Fixture> fixtureList = body.FixtureList;
            for (int index = 0, length = fixtureList.Count; index < length; index++) {
                GenericShapeClone2D shapeClone = poolClone2D.GetNew();
                shapeClone.Clone(body.FixtureList[index].Shape);

                this.shapesClone.Add(shapeClone);
            }
        }

		public void Restore(Physics2D.Body body) {
            body._bodyType = this._bodyType;
            body._sleepingAllowed = this._sleepingAllowed;
            body._awake = this._awake;
            body._enabled = this._enabled;
            body._sleepTime = this._sleepTime;

            body._xf.p = this._xf.p;
            body._xf.q = this._xf.q;
            
            bool lastDisabled = body.disabled;
            body.disabled = this.disabled;

            if (lastDisabled && !body.disabled) {
                //Physics2D.ContactManager.physicsManager.GetGameObject(body).SetActive(true);
            }
            
            List<Physics2D.Fixture> fixtureList = body.FixtureList;
            for (int index = 0, length = this.shapesClone.Count; index < length; index++) {
                this.shapesClone[index].Restore(fixtureList[index].Shape);
            }
        }

    }

}