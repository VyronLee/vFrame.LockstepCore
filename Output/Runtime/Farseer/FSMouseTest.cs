using UnityEngine;
using vFrame.Lockstep.Core;
using vFrame.Lockstep.Core.FarseerPhysics.Collision;
using vFrame.Lockstep.Core.FarseerPhysics.Collision.Shapes;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics;
using vFrame.Lockstep.Core.FarseerPhysics.Dynamics.Joints;
using vFrame.Lockstep.Core.FarseerPhysics.Factories;
using Transform = vFrame.Lockstep.Core.FarseerPhysics.Common.Transform;

[AddComponentMenu("FarseerUnity/Debug/Mouse Test Component")]
public class FSMouseTest : MonoBehaviour {

	protected FixedMouseJoint mouseJoint;

	public virtual void Update()
	{
		UpdateMouseWorld();
		MouseDrag();
	}

	static public float MouseXWorldPhys = 0f;
	static public float MouseYWorldPhys = 0f;
	public virtual void UpdateMouseWorld()
	{
		Vector3 wp = Camera.main.ScreenToWorldPoint(Input.mousePosition);
		MouseXWorldPhys = wp.x;// -parent.position.x;
		MouseYWorldPhys = wp.y;// - parent.position.y;
		//Debug.Log("MX: " + MouseXWorldPhys + " MY: " + MouseYWorldPhys);
		//dynB.Position = new FVector2(MouseXWorldPhys, MouseYWorldPhys);
	}

	protected TSVector2 mousePVec = new TSVector2();
	public virtual Body GetBodyAtMouse()
	{
		return GetBodyAtMouse(false);
	}
	public virtual Body GetBodyAtMouse(bool includeStatic)
	{
		// Make a small box
		mousePVec.x = MouseXWorldPhys;
		mousePVec.y = MouseYWorldPhys;
		TSVector2 lowerBound = new TSVector2(MouseXWorldPhys - 0.001f, MouseYWorldPhys - 0.001f);
		TSVector2 upperBound = new TSVector2(MouseXWorldPhys + 0.001f, MouseYWorldPhys + 0.001f);
		AABB aabb = new AABB(lowerBound, upperBound);
		Body body = null;

		// Query the world for overlapping shapes
		System.Func<Fixture, bool> GetBodyCallback = delegate (Fixture fixture0)
		{
			Shape shape = fixture0.Shape;
			if(fixture0.Body.BodyType != BodyType.Static || includeStatic)
			{
				Transform transform0;
				fixture0.Body.GetTransform(out transform0);
				bool inside = shape.TestPoint(ref transform0, ref mousePVec);
				if(inside)
				{
					body = fixture0.Body;
					return false;
				}
			}
			return true;
		};
		FSWorldComponent.PhysicsWorld.QueryAABB(GetBodyCallback, ref aabb);
		return body;
	}

	public virtual void MouseDrag()
	{
		// mouse press
		if(Input.GetMouseButtonDown(0) && mouseJoint == null)
		{
			Body body = GetBodyAtMouse();
			if(body != null)
			{
				TSVector2 target = new TSVector2(MouseXWorldPhys, MouseYWorldPhys);
				mouseJoint = JointFactory.CreateFixedMouseJoint(FSWorldComponent.PhysicsWorld, body, target);
				mouseJoint.CollideConnected = true;
				mouseJoint.MaxForce = 300f * body.Mass;
				body.Awake = true;
			}
		}
		// mouse release
		if(Input.GetMouseButtonUp(0))
		{
			if(mouseJoint != null)
			{
				FSWorldComponent.PhysicsWorld.RemoveJoint(mouseJoint);
				mouseJoint = null;
			}
		}

		// mouse move
		if(mouseJoint != null)
		{
			TSVector2 p2 = new TSVector2(MouseXWorldPhys, MouseYWorldPhys);
			mouseJoint.WorldAnchorB = p2;
		}
	}
}
