using System;
using System.Collections.Generic;

namespace vFrame.Lockstep.Core.Physics2D
{
    public struct PhyHitInfo
    {
        public PhyGameObject go;
        public TSVector point;
        public TSVector normal;
        public FP distance;
    }


    class PhyRaycastResultCallback
    {
        #region 输入
        private List<PhyHitInfo> m_rayallList;
        private FP m_height;
        private FP m_dist;

        #endregion

        public Func<Fixture, TSVector2, TSVector2, FP, FP> m_raycastCallback;
        public Func<Fixture, TSVector2, TSVector2, FP, FP> m_raycastAllCallback;
        public Func<Fixture, TSVector2, TSVector2, FP, FP> m_circlecastCallback;
        public PhyHitInfo m_hit = new PhyHitInfo();

        public PhyRaycastResultCallback()
        {
            m_raycastCallback = RaycastCallback;
            m_raycastAllCallback = RaycastAllCallback;
            m_circlecastCallback = RaycastCallback;
        }

        public void InitRaycastAll(FP height, FP distance, List<PhyHitInfo> listResult)
        {
            m_height = height;
            m_dist = distance;
            m_rayallList = listResult;
            m_hit.go = null;
        }

        public void InitRaycast(FP height, FP distance)
        {
            m_height = height;
            m_dist = distance;
            m_rayallList = null;
            m_hit.go = null;
        }

        public void InitCircleCast(FP height, FP distance)
        {
            InitRaycast(height, distance);
        }

        private FP RaycastCallback(Fixture f, TSVector2 p, TSVector2 n, FP fr)
        {
            var go = (PhyGameObject)f.Body.UserData;
            if (go != null)
            {
                m_hit.go = go;
                m_hit.point = PhyUtil.PhyVed2ToVec3(p, m_height);
                m_hit.normal = PhyUtil.PhyVed2ToVec3(n);
                m_hit.distance = fr* m_dist;
            }

            return fr;
        }

        private FP RaycastAllCallback(Fixture f, TSVector2 p, TSVector2 n, FP fr)
        {
            PhyGameObject go = f.Body.UserData as PhyGameObject;
            if (go == null)
            {
                return 1;
            }

            var hitInfo = new PhyHitInfo();
            hitInfo.go = go;
            hitInfo.normal = PhyUtil.PhyVed2ToVec3(n);
            hitInfo.point = PhyUtil.PhyVed2ToVec3(p, m_height);
            hitInfo.distance = fr * m_dist;
            m_rayallList.Add(hitInfo);

            return 1;
        }

    }

    /// <summary>
    /// 封装更加易用的接口,接受3纬的坐标，内部转换为2纬的接口
    /// </summary>
    public class PhyWorld
    {
        private World m_world;
        private PhyGameObject m_ground;

        public PhyWorld(int groundLayer)
        {
            m_world = new World(new TSVector2(0,0));
            m_ground = new PhyGameObject("ground");
            m_ground.layer = groundLayer;
        }

        public void Destroy()
        {
            m_world.Clear();
        }

        #region 物理世界构建

        public PhyGameObject AddMergeShape(MergeShape mergeShape)
        {
            Shape shape = null;
            if (mergeShape.m_loop)
            {
                shape = new PolygonShape(mergeShape.m_vertex);

            }
            else
            {
                ///强制全部转换为边
                if (mergeShape.EdgeCount == 1)
                {
                    shape = new EdgeShape(mergeShape.m_vertex[0], mergeShape.m_vertex[1]);
                }
                else
                {
                    shape = new ChainShape(mergeShape.m_vertex, false);
                }
            }

            return AddShape(mergeShape.m_name, mergeShape.m_center, mergeShape.m_forward, shape, BodyType.Static, mergeShape.m_layer);
        }

        public PhyGameObject AddBoxCollider(string name, TSVector2 centerPos, TSVector2 forward, TSVector2 size,
            int layer, BodyType bodyType = BodyType.Static)
        {
            size *= FP.Half;

            var shape =  new PolygonShape(PolygonTools.CreateRectangle(size.x, size.y));
            return AddShape(name, centerPos, forward, shape, bodyType, layer);
        }

        public PhyGameObject AddCircleCollider(string name, TSVector2 centerPos, TSVector2 forward, FP radius,
            int layer, BodyType bodyType = BodyType.Static)
        {
            var shape = new CircleShape(radius);
            return AddShape(name, centerPos, forward, shape, bodyType, layer);
        }

        #endregion

        PhyRaycastResultCallback m_castResult = new PhyRaycastResultCallback();

        #region 碰撞检测
        public bool RayCastAll(List<PhyHitInfo> listResult, TSVector point1, TSVector dir, FP dist, int layerMask)
        {
            listResult.Clear();

            TSVector2 p1 = new TSVector2(point1.x, point1.z);
            TSVector2 dir2 = new TSVector2(dir.x, dir.z);
            dir2.Normalize();

            m_castResult.InitRaycastAll(point1.y, dist, listResult);

            TSVector2 p2 = p1 + dir2 * dist;
            m_world.RayCast(m_castResult.m_raycastAllCallback, p1, p2, layerMask);

            return listResult.Count > 0;
        }

        public bool RayCast(TSVector point1, TSVector dir, FP dist, int layerMask, out PhyHitInfo hit)
        {
            TSVector2 p1 = new TSVector2(point1.x, point1.z);
            TSVector2 dir2 = new TSVector2(dir.x, dir.z);
            dir2.Normalize();

            var hitWall = RayCast(p1, point1.y, dir2, dist, layerMask, out hit);
            var ySpeed = -dir.y ;
            if (ySpeed > FP.EN5)
            {
                ///那么肯定相撞了
                if (point1.y < ySpeed * dist)
                {
                    var moveDist = point1.y / ySpeed;
                    if (moveDist < FP.Zero)
                    {
                        moveDist = FP.Zero;
                    }

                    ///如果墙壁没有碰撞或者更远，那么就取地面这个
                    if (!hitWall || hit.distance > moveDist)
                    {
                        hitWall = true;
                        hit.distance = moveDist;
                        hit.point = point1 + dir * moveDist;
                        hit.normal = TSVector.up;
                        hit.go = m_ground;
                    }
                }
            }

            return hitWall;
        }

        public bool RayCast(TSVector2 p1, FP y, TSVector2 dir2, FP dist, int layerMask, out PhyHitInfo hit)
        {
            TSVector2 p2 = p1 + dir2 * dist;

            m_castResult.InitRaycast(y, dist);
            m_world.RayCast(m_castResult.m_raycastCallback, p1, p2, layerMask);

            hit = m_castResult.m_hit;
            return hit.go != null;
        }

        /// <summary>
        /// 圆形检测
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <param name="radius"></param>
        /// <returns></returns>
        public bool CircleCast(TSVector2 p1, FP y, FP radius, TSVector2 dir, FP dist,  int layerMask, out PhyHitInfo hit)
        {
            m_castResult.InitCircleCast(y, dist);
            m_world.CircleCast(m_castResult.m_circlecastCallback, p1, dir, dist, radius, layerMask);

            hit = m_castResult.m_hit;
            return hit.go != null;
        }

        public bool CircleCast(TSVector point1, FP radius, TSVector dir, FP dist, int layerMask, out PhyHitInfo hit)
        {
            TSVector2 p1 = new TSVector2(point1.x, point1.z);
            TSVector2 dir2 = new TSVector2(dir.x, dir.z);
            dir2.Normalize();

            var hitWall = CircleCast(p1, point1.y, radius, dir2, dist, layerMask, out hit);
            var ySpeed = -dir.y;
            if (ySpeed > FP.EN5)
            {
                ///那么肯定相撞了
                if (point1.y < ySpeed * dist)
                {
                    var moveDist = point1.y / ySpeed;
                    if (moveDist < FP.Zero)
                    {
                        moveDist = FP.Zero;
                    }

                    ///如果墙壁没有碰撞或者更远，那么就取地面这个
                    if (!hitWall || hit.distance > moveDist)
                    {
                        hitWall = true;
                        hit.distance = moveDist;
                        hit.point = point1 + dir * moveDist;
                        hit.normal = TSVector.up;
                        hit.go = m_ground;
                    }
                }
            }

            return hitWall;
        }


        /// <summary>
        /// 快速版本，不准确，按照三条射线比较粗放的检测,主要用来检测是否可以释放npc等
        /// </summary>
        /// <param name="context"></param>
        /// <param name="position"></param>
        /// <param name="radius"></param>
        /// <param name="inDirNormal"></param>
        /// <param name="dist"></param>
        /// <param name="maxDist"></param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        public bool CircleCastQuick(TSVector position,
            FP radius, TSVector inDirNormal, out FP dist, FP maxDist, int layerMask)
        {
            ///都按照最长的距离来判断

            bool hasHit = false;
            FP hitDistance = maxDist + radius;

            PhyHitInfo hitInfo;
            if (RayCast(position, inDirNormal, hitDistance, layerMask, out hitInfo))
            {
                hitDistance = hitInfo.distance;
                hasHit = true;
            }

            var leftPoint = position + TSQuaternion.Left90Rotate * inDirNormal;
            if (RayCast(leftPoint, inDirNormal, hitDistance, layerMask, out hitInfo))
            {
                hitDistance = hitInfo.distance;
                hasHit = true;
            }

            var rightPoint = position + TSQuaternion.Right90Rotate * inDirNormal;
            if (RayCast(rightPoint, inDirNormal, hitDistance, layerMask, out hitInfo))
            {
                hitDistance = hitInfo.distance;
                hasHit = true;
            }

            if (hasHit)
            {
                dist = hitDistance;
            }
            else
            {
                dist = FP.Zero;
            }

            return hasHit;
        }

        #endregion


        private PhyGameObject AddShape(string name, TSVector2 centerPos, TSVector2 forward, Shape shape, BodyType bodyType, int layer)
        {
            var go = new PhyGameObject(name);
            var body = BodyFactory.CreateBody(m_world, centerPos, forward, go);
            go.body = body;

            body.CreateFixture(shape);
            body.BodyType = bodyType;

            body.IsSensor = false;
            body.CollisionCategories = (Category) (1 << layer);

            go.layer = layer;
            m_world.ProcessAddedBodies();
            return go;
        }

        public void RemoveGameObject(PhyGameObject go)
        {
            if (go == null || go.body == null)
            {
                return;
            }
            m_world.RemoveBody(go.body);
            m_world.ProcessRemovedBodies();
        }

#if DOD_DEBUG
        public void DrawGizmoz()
        {
            m_world.DrawGizmoz();
        }

#endif
    }
}