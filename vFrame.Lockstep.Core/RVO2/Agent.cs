/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System.Collections.Generic;

namespace vFrame.Lockstep.Core.RVO2
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal class Agent
    {
        internal IList<KeyValuePair<FixedPoint, Agent>> agentNeighbors_ = new List<KeyValuePair<FixedPoint, Agent>>();
        internal IList<KeyValuePair<FixedPoint, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<FixedPoint, Obstacle>>();
        internal IList<Line> orcaLines_ = new List<Line>();
        internal TSVector2 position_;
        internal TSVector2 prefVelocity_;
        internal TSVector2 velocity_;
        internal int id_ = 0;
        internal int maxNeighbors_ = 0;
        internal FixedPoint maxSpeed_ = 0.0f;
        internal FixedPoint neighborDist_ = 0.0f;
        internal FixedPoint radius_ = 0.0f;
        internal FixedPoint timeHorizon_ = 0.0f;
        internal FixedPoint timeHorizonObst_ = 0.0f;

        private TSVector2 newVelocity_;

        private Simulator simulator_;

        public Agent(Simulator simulator) {
            simulator_ = simulator;
        }

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors_.Clear();
            FixedPoint rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
            simulator_.kdTree_.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors_.Clear();

            if (maxNeighbors_ > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist_);
                simulator_.kdTree_.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void computeNewVelocity()
        {
            orcaLines_.Clear();

            FixedPoint invTimeHorizonObst = 1.0f / timeHorizonObst_;

            /* Create obstacle ORCA lines. */
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {

                Obstacle obstacle1 = obstacleNeighbors_[i].Value;
                Obstacle obstacle2 = obstacle1.next_;

                TSVector2 relativePosition1 = obstacle1.point_ - position_;
                TSVector2 relativePosition2 = obstacle2.point_ - position_;

                /*
                 * Check if velocity obstacle of obstacle is already taken care
                 * of by previously constructed obstacle ORCA lines.
                 */
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines_.Count; ++j)
                {
                    if (RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON && RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                FixedPoint distSq1 = RVOMath.absSq(relativePosition1);
                FixedPoint distSq2 = RVOMath.absSq(relativePosition2);

                FixedPoint radiusSq = RVOMath.sqr(radius_);

                TSVector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
                FixedPoint s = (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector);
                FixedPoint distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

                Line line;

                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (obstacle1.convex_)
                    {
                        line.point = new TSVector2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new TSVector2(-1 * relativePosition1.y, relativePosition1.x));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (obstacle2.convex_ && RVOMath.det(relativePosition2, obstacle2.direction_) >= 0.0f)
                    {
                        line.point = new TSVector2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new TSVector2(-1 * relativePosition2.y, relativePosition2.x));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.point = new TSVector2(0.0f, 0.0f);
                    line.direction = -obstacle1.direction_;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                TSVector2 leftLegDirection, rightLegDirection;

                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!obstacle1.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    FixedPoint leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                    leftLegDirection = new TSVector2(relativePosition1.x * leg1 - relativePosition1.y * radius_, relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                    rightLegDirection = new TSVector2(relativePosition1.x * leg1 + relativePosition1.y * radius_, -relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                }
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!obstacle2.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    FixedPoint leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new TSVector2(relativePosition2.x * leg2 - relativePosition2.y * radius_, relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                    rightLegDirection = new TSVector2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (obstacle1.convex_)
                    {
                        FixedPoint leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new TSVector2(relativePosition1.x * leg1 - relativePosition1.y * radius_, relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1.direction_;
                    }

                    if (obstacle2.convex_)
                    {
                        FixedPoint leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new TSVector2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1.direction_;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                Obstacle leftNeighbor = obstacle1.previous_;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex_ && RVOMath.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -leftNeighbor.direction_;
                    isLeftLegForeign = true;
                }

                if (obstacle2.convex_ && RVOMath.det(rightLegDirection, obstacle2.direction_) <= 0.0f)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = obstacle2.direction_;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                TSVector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
                TSVector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
                TSVector2 cutOffVector = rightCutOff - leftCutOff;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                FixedPoint t = obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutOff) * cutOffVector) / RVOMath.absSq(cutOffVector);
                FixedPoint tLeft = (velocity_ - leftCutOff) * leftLegDirection;
                FixedPoint tRight = (velocity_ - rightCutOff) * rightLegDirection;

                if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* Project on left cut-off circle. */
                    TSVector2 unitW = RVOMath.normalize(velocity_ - leftCutOff);

                    line.direction = new TSVector2(unitW.y, -unitW.x);
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* Project on right cut-off circle. */
                    TSVector2 unitW = RVOMath.normalize(velocity_ - rightCutOff);

                    line.direction = new TSVector2(unitW.y, -unitW.x);
                    line.point = rightCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                FixedPoint distSqCutoff = (t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? FixedPoint.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + t * cutOffVector));
                FixedPoint distSqLeft = tLeft < 0.0f ? FixedPoint.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + tLeft * leftLegDirection));
                FixedPoint distSqRight = tRight < 0.0f ? FixedPoint.PositiveInfinity : RVOMath.absSq(velocity_ - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.direction = -obstacle1.direction_;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new TSVector2(-line.direction.y, line.direction.x);
                    orcaLines_.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new TSVector2(-line.direction.y, line.direction.x);
                    orcaLines_.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + radius_ * invTimeHorizonObst * new TSVector2(-line.direction.y, line.direction.x);
                orcaLines_.Add(line);
            }

            int numObstLines = orcaLines_.Count;

            FixedPoint invTimeHorizon = 1.0f / timeHorizon_;

            /* Create agent ORCA lines. */
            for (int i = 0; i < agentNeighbors_.Count; ++i)
            {
                Agent other = agentNeighbors_[i].Value;

                TSVector2 relativePosition = other.position_ - position_;
                TSVector2 relativeVelocity = velocity_ - other.velocity_;
                FixedPoint distSq = RVOMath.absSq(relativePosition);
                FixedPoint combinedRadius = radius_ + other.radius_;
                FixedPoint combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                TSVector2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    TSVector2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    FixedPoint wLengthSq = RVOMath.absSq(w);
                    FixedPoint dotProduct1 = w * relativePosition;

                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        FixedPoint wLength = RVOMath.sqrt(wLengthSq);
                        TSVector2 unitW = w / wLength;

                        line.direction = new TSVector2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        FixedPoint leg = RVOMath.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            line.direction = new TSVector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            line.direction = -new TSVector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        FixedPoint dotProduct2 = relativeVelocity * line.direction;
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    FixedPoint invTimeStep = 1.0f / simulator_.timeStep_;

                    /* Vector from cutoff center to relative velocity. */
                    TSVector2 w = relativeVelocity - invTimeStep * relativePosition;

                    FixedPoint wLength = RVOMath.abs(w);
                    TSVector2 unitW = w / wLength;

                    line.direction = new TSVector2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = velocity_ + 0.5f * u;
                orcaLines_.Add(line);
            }

            int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref FixedPoint rangeSq)
        {
            if (this != agent)
            {
                FixedPoint distSq = RVOMath.absSq(position_ - agent.position_);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors_.Count < maxNeighbors_)
                    {
                        agentNeighbors_.Add(new KeyValuePair<FixedPoint, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors_.Count - 1;

                    while (i != 0 && distSq < agentNeighbors_[i - 1].Key)
                    {
                        agentNeighbors_[i] = agentNeighbors_[i - 1];
                        --i;
                    }

                    agentNeighbors_[i] = new KeyValuePair<FixedPoint, Agent>(distSq, agent);

                    if (agentNeighbors_.Count == maxNeighbors_)
                    {
                        rangeSq = agentNeighbors_[agentNeighbors_.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, FixedPoint rangeSq)
        {
            Obstacle nextObstacle = obstacle.next_;

            FixedPoint distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            if (distSq < rangeSq)
            {
                obstacleNeighbors_.Add(new KeyValuePair<FixedPoint, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors_.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors_[i - 1].Key)
                {
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                obstacleNeighbors_[i] = new KeyValuePair<FixedPoint, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            velocity_ = newVelocity_;
            position_ += velocity_ * simulator_.timeStep_;
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, FixedPoint radius, TSVector2 optVelocity, bool directionOpt, ref TSVector2 result)
        {
            FixedPoint dotProduct = lines[lineNo].point * lines[lineNo].direction;
            FixedPoint discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            FixedPoint sqrtDiscriminant = RVOMath.sqrt(discriminant);
            FixedPoint tLeft = -dotProduct - sqrtDiscriminant;
            FixedPoint tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                FixedPoint denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                FixedPoint numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                FixedPoint t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = TSMath.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = TSMath.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (optVelocity * lines[lineNo].direction > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                FixedPoint t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram2(IList<Line> lines, FixedPoint radius, TSVector2 optVelocity, bool directionOpt, ref TSVector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    TSVector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, FixedPoint radius, ref TSVector2 result)
        {
            FixedPoint distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        FixedPoint determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].direction * lines[j].direction > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    TSVector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new TSVector2(-lines[i].direction.y, lines[i].direction.x), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}