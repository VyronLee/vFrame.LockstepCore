/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
*
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution.
*/

using System;

namespace vFrame.Lockstep.Core
{
    /// <summary>
    /// A Quaternion representing an orientation.
    /// </summary>
    [Serializable]
    public struct TSQuaternion
    {
        /// <summary>The X component of the quaternion.</summary>
        public FixedPoint x;

        /// <summary>The Y component of the quaternion.</summary>
        public FixedPoint y;

        /// <summary>The Z component of the quaternion.</summary>
        public FixedPoint z;

        /// <summary>The W component of the quaternion.</summary>
        public FixedPoint w;

        public static readonly TSQuaternion identity;

        static TSQuaternion() {
            identity = new TSQuaternion(0, 0, 0, 1);
        }

        /// <summary>
        /// Initializes a new instance of the JQuaternion structure.
        /// </summary>
        /// <param name="x">The X component of the quaternion.</param>
        /// <param name="y">The Y component of the quaternion.</param>
        /// <param name="z">The Z component of the quaternion.</param>
        /// <param name="w">The W component of the quaternion.</param>
        public TSQuaternion(FixedPoint x, FixedPoint y, FixedPoint z, FixedPoint w) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public void Set(FixedPoint new_x, FixedPoint new_y, FixedPoint new_z, FixedPoint new_w) {
            x = new_x;
            y = new_y;
            z = new_z;
            w = new_w;
        }

        public void SetFromToRotation(TSVector fromDirection, TSVector toDirection) {
            var targetRotation = FromToRotation(fromDirection, toDirection);
            Set(targetRotation.x, targetRotation.y, targetRotation.z, targetRotation.w);
        }

        public TSVector eulerAngles {
            get {
                var result = new TSVector();

                var ysqr = y * y;
                var t0 = -2 * (ysqr + z * z) + FixedPoint.One;
                var t1 = +2 * (x * y - w * z);
                var t2 = -2 * (x * z + w * y);
                var t3 = +2 * (y * z - w * x);
                var t4 = -2 * (x * x + ysqr) + FixedPoint.One;

                t2 = t2 > FixedPoint.One ? FixedPoint.One : t2;
                t2 = t2 < -1 ? -1 : t2;

                result.x = FixedPoint.Atan2(t3, t4) * FixedPoint.Rad2Deg;
                result.y = FixedPoint.Asin(t2) * FixedPoint.Rad2Deg;
                result.z = FixedPoint.Atan2(t1, t0) * FixedPoint.Rad2Deg;

                return result * -1;
            }
        }

        public static FixedPoint Angle(TSQuaternion a, TSQuaternion b) {
            var aInv = Inverse(a);
            var f = b * aInv;

            var angle = FixedPoint.Acos(f.w) * 2 * FixedPoint.Rad2Deg;

            if (angle > 180) angle = 360 - angle;

            return angle;
        }

        /// <summary>
        /// Quaternions are added.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The sum of both quaternions.</returns>

        #region public static JQuaternion Add(JQuaternion quaternion1, JQuaternion quaternion2)

        public static TSQuaternion Add(TSQuaternion quaternion1, TSQuaternion quaternion2) {
            TSQuaternion result;
            Add(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        public static TSQuaternion LookRotation(TSVector forward) {
            return CreateFromMatrix(TSMatrix.LookAt(forward, TSVector.up));
        }

        public static TSQuaternion LookRotation(TSVector forward, TSVector upwards) {
            return CreateFromMatrix(TSMatrix.LookAt(forward, upwards));
        }

        public static TSQuaternion Slerp(TSQuaternion from, TSQuaternion to, FixedPoint t) {
            t = TSMath.Clamp(t, 0, 1);

            var dot = Dot(from, to);

            if (dot < FixedPoint.Zero) {
                to = Multiply(to, -1);
                dot = -dot;
            }

            var halfTheta = FixedPoint.Acos(dot);

            return Multiply(
                Multiply(from, FixedPoint.FastSin((1 - t) * halfTheta)) + Multiply(to, FixedPoint.FastSin(t * halfTheta)),
                1 / FixedPoint.FastSin(halfTheta));
        }

        public static TSQuaternion RotateTowards(TSQuaternion from, TSQuaternion to, FixedPoint maxDegreesDelta) {
            var dot = Dot(from, to);

            if (dot < FixedPoint.Zero) {
                to = Multiply(to, -1);
                dot = -dot;
            }

            var halfTheta = FixedPoint.Acos(dot);
            var theta = halfTheta * 2;

            maxDegreesDelta *= FixedPoint.Deg2Rad;

            if (maxDegreesDelta >= theta) return to;

            maxDegreesDelta /= theta;

            return Multiply(
                Multiply(from, FixedPoint.FastSin((1 - maxDegreesDelta) * halfTheta)) +
                Multiply(to, FixedPoint.FastSin(maxDegreesDelta * halfTheta)), 1 / FixedPoint.FastSin(halfTheta));
        }

        public static TSQuaternion Euler(FixedPoint x, FixedPoint y, FixedPoint z) {
            x *= FixedPoint.Deg2Rad;
            y *= FixedPoint.Deg2Rad;
            z *= FixedPoint.Deg2Rad;

            TSQuaternion rotation;
            CreateFromYawPitchRoll(y, x, z, out rotation);

            return rotation;
        }

        public static TSQuaternion Euler(TSVector eulerAngles) {
            return Euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        }

        public static TSQuaternion AngleAxis(FixedPoint angle, TSVector axis) {
            axis = axis * FixedPoint.Deg2Rad;
            axis.Normalize();

            var halfAngle = angle * FixedPoint.Deg2Rad * FixedPoint.Half;

            TSQuaternion rotation;
            var sin = FixedPoint.FastSin(halfAngle);

            rotation.x = axis.x * sin;
            rotation.y = axis.y * sin;
            rotation.z = axis.z * sin;
            rotation.w = FixedPoint.FastCos(halfAngle);

            return rotation;
        }

        public static void CreateFromYawPitchRoll(FixedPoint yaw, FixedPoint pitch, FixedPoint roll,
            out TSQuaternion result) {
            var num9 = roll * FixedPoint.Half;
            var num6 = FixedPoint.FastSin(num9);
            var num5 = FixedPoint.FastCos(num9);
            var num8 = pitch * FixedPoint.Half;
            var num4 = FixedPoint.FastSin(num8);
            var num3 = FixedPoint.FastCos(num8);
            var num7 = yaw * FixedPoint.Half;
            var num2 = FixedPoint.FastSin(num7);
            var num = FixedPoint.FastCos(num7);
            result.x = num * num4 * num5 + num2 * num3 * num6;
            result.y = num2 * num3 * num5 - num * num4 * num6;
            result.z = num * num3 * num6 - num2 * num4 * num5;
            result.w = num * num3 * num5 + num2 * num4 * num6;
        }

        /// <summary>
        /// Quaternions are added.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The sum of both quaternions.</param>
        public static void Add(ref TSQuaternion quaternion1, ref TSQuaternion quaternion2, out TSQuaternion result) {
            result.x = quaternion1.x + quaternion2.x;
            result.y = quaternion1.y + quaternion2.y;
            result.z = quaternion1.z + quaternion2.z;
            result.w = quaternion1.w + quaternion2.w;
        }

        #endregion

        public static TSQuaternion Conjugate(TSQuaternion value) {
            TSQuaternion quaternion;
            quaternion.x = -value.x;
            quaternion.y = -value.y;
            quaternion.z = -value.z;
            quaternion.w = value.w;
            return quaternion;
        }

        public static FixedPoint Dot(TSQuaternion a, TSQuaternion b) {
            return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        }

        public static TSQuaternion Inverse(TSQuaternion rotation) {
            var invNorm = FixedPoint.One / (rotation.x * rotation.x + rotation.y * rotation.y +
                                            rotation.z * rotation.z + rotation.w * rotation.w);
            return Multiply(Conjugate(rotation), invNorm);
        }

        public static TSQuaternion FromToRotation(TSVector fromVector, TSVector toVector) {
            var w = TSVector.Cross(fromVector, toVector);
            var q = new TSQuaternion(w.x, w.y, w.z, TSVector.Dot(fromVector, toVector));
            q.w += FixedPoint.Sqrt(fromVector.sqrMagnitude * toVector.sqrMagnitude);
            q.Normalize();

            return q;
        }

        public static TSQuaternion Lerp(TSQuaternion a, TSQuaternion b, FixedPoint t) {
            t = TSMath.Clamp(t, FixedPoint.Zero, FixedPoint.One);

            return LerpUnclamped(a, b, t);
        }

        public static TSQuaternion LerpUnclamped(TSQuaternion a, TSQuaternion b, FixedPoint t) {
            var result = Multiply(a, 1 - t) + Multiply(b, t);
            result.Normalize();

            return result;
        }

        public void ToAngleAxis(out FixedPoint angle, out TSVector axis)
        {
            ToAxisAngleRad(this, out axis, out angle);
            angle *= TSMath.Rad2Deg;
        }

        private static void ToAxisAngleRad(TSQuaternion q, out TSVector axis, out FixedPoint angle) {
            if (TSMath.Abs(q.w) > 1.0f)
                q.Normalize();
            angle = 2.0f * TSMath.Acos(q.w); // angle
            var den = TSMath.Sqrt(1.0f - q.w * q.w);
            if (den > 0.0001f)
                axis = new TSVector(q.x, q.y, q.z) / den;
            else
                // This occurs when the angle is zero.
                // Not a problem: just set an arbitrary normalized axis.
                axis = new TSVector(1, 0, 0);
        }

        /// <summary>
        /// Quaternions are subtracted.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The difference of both quaternions.</returns>

        #region public static JQuaternion Subtract(JQuaternion quaternion1, JQuaternion quaternion2)

        public static TSQuaternion Subtract(TSQuaternion quaternion1, TSQuaternion quaternion2) {
            TSQuaternion result;
            Subtract(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        /// <summary>
        /// Quaternions are subtracted.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The difference of both quaternions.</param>
        public static void Subtract(ref TSQuaternion quaternion1, ref TSQuaternion quaternion2,
            out TSQuaternion result) {
            result.x = quaternion1.x - quaternion2.x;
            result.y = quaternion1.y - quaternion2.y;
            result.z = quaternion1.z - quaternion2.z;
            result.w = quaternion1.w - quaternion2.w;
        }

        #endregion

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The product of both quaternions.</returns>

        #region public static JQuaternion Multiply(JQuaternion quaternion1, JQuaternion quaternion2)

        public static TSQuaternion Multiply(TSQuaternion quaternion1, TSQuaternion quaternion2) {
            TSQuaternion result;
            Multiply(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The product of both quaternions.</param>
        public static void Multiply(ref TSQuaternion quaternion1, ref TSQuaternion quaternion2,
            out TSQuaternion result) {
            var x = quaternion1.x;
            var y = quaternion1.y;
            var z = quaternion1.z;
            var w = quaternion1.w;
            var num4 = quaternion2.x;
            var num3 = quaternion2.y;
            var num2 = quaternion2.z;
            var num = quaternion2.w;
            var num12 = y * num2 - z * num3;
            var num11 = z * num4 - x * num2;
            var num10 = x * num3 - y * num4;
            var num9 = x * num4 + y * num3 + z * num2;
            result.x = x * num + num4 * w + num12;
            result.y = y * num + num3 * w + num11;
            result.z = z * num + num2 * w + num10;
            result.w = w * num - num9;
        }

        #endregion

        /// <summary>
        /// Scale a quaternion
        /// </summary>
        /// <param name="quaternion1">The quaternion to scale.</param>
        /// <param name="scaleFactor">Scale factor.</param>
        /// <returns>The scaled quaternion.</returns>

        #region public static JQuaternion Multiply(JQuaternion quaternion1, FP scaleFactor)

        public static TSQuaternion Multiply(TSQuaternion quaternion1, FixedPoint scaleFactor) {
            TSQuaternion result;
            Multiply(ref quaternion1, scaleFactor, out result);
            return result;
        }

        /// <summary>
        /// Scale a quaternion
        /// </summary>
        /// <param name="quaternion1">The quaternion to scale.</param>
        /// <param name="scaleFactor">Scale factor.</param>
        /// <param name="result">The scaled quaternion.</param>
        public static void Multiply(ref TSQuaternion quaternion1, FixedPoint scaleFactor, out TSQuaternion result) {
            result.x = quaternion1.x * scaleFactor;
            result.y = quaternion1.y * scaleFactor;
            result.z = quaternion1.z * scaleFactor;
            result.w = quaternion1.w * scaleFactor;
        }

        #endregion

        /// <summary>
        /// Sets the length of the quaternion to one.
        /// </summary>

        #region public void Normalize()

        public void Normalize() {
            var num2 = x * x + y * y + z * z + w * w;
            var num = 1 / FixedPoint.Sqrt(num2);
            x *= num;
            y *= num;
            z *= num;
            w *= num;
        }

        #endregion

        /// <summary>
        /// Creates a quaternion from a matrix.
        /// </summary>
        /// <param name="matrix">A matrix representing an orientation.</param>
        /// <returns>JQuaternion representing an orientation.</returns>

        #region public static JQuaternion CreateFromMatrix(JMatrix matrix)

        public static TSQuaternion CreateFromMatrix(TSMatrix matrix) {
            TSQuaternion result;
            CreateFromMatrix(ref matrix, out result);
            return result;
        }

        /// <summary>
        /// Creates a quaternion from a matrix.
        /// </summary>
        /// <param name="matrix">A matrix representing an orientation.</param>
        /// <param name="result">JQuaternion representing an orientation.</param>
        public static void CreateFromMatrix(ref TSMatrix matrix, out TSQuaternion result) {
            var num8 = matrix.M11 + matrix.M22 + matrix.M33;
            if (num8 > FixedPoint.Zero) {
                var num = FixedPoint.Sqrt(num8 + FixedPoint.One);
                result.w = num * FixedPoint.Half;
                num = FixedPoint.Half / num;
                result.x = (matrix.M23 - matrix.M32) * num;
                result.y = (matrix.M31 - matrix.M13) * num;
                result.z = (matrix.M12 - matrix.M21) * num;
            }
            else if (matrix.M11 >= matrix.M22 && matrix.M11 >= matrix.M33) {
                var num7 = FixedPoint.Sqrt(FixedPoint.One + matrix.M11 - matrix.M22 - matrix.M33);
                var num4 = FixedPoint.Half / num7;
                result.x = FixedPoint.Half * num7;
                result.y = (matrix.M12 + matrix.M21) * num4;
                result.z = (matrix.M13 + matrix.M31) * num4;
                result.w = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33) {
                var num6 = FixedPoint.Sqrt(FixedPoint.One + matrix.M22 - matrix.M11 - matrix.M33);
                var num3 = FixedPoint.Half / num6;
                result.x = (matrix.M21 + matrix.M12) * num3;
                result.y = FixedPoint.Half * num6;
                result.z = (matrix.M32 + matrix.M23) * num3;
                result.w = (matrix.M31 - matrix.M13) * num3;
            }
            else {
                var num5 = FixedPoint.Sqrt(FixedPoint.One + matrix.M33 - matrix.M11 - matrix.M22);
                var num2 = FixedPoint.Half / num5;
                result.x = (matrix.M31 + matrix.M13) * num2;
                result.y = (matrix.M32 + matrix.M23) * num2;
                result.z = FixedPoint.Half * num5;
                result.w = (matrix.M12 - matrix.M21) * num2;
            }
        }

        #endregion

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The product of both quaternions.</returns>

        #region public static FP operator *(JQuaternion value1, JQuaternion value2)

        public static TSQuaternion operator *(TSQuaternion value1, TSQuaternion value2) {
            TSQuaternion result;
            Multiply(ref value1, ref value2, out result);
            return result;
        }

        #endregion

        /// <summary>
        /// Add two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The sum of both quaternions.</returns>

        #region public static FP operator +(JQuaternion value1, JQuaternion value2)

        public static TSQuaternion operator +(TSQuaternion value1, TSQuaternion value2) {
            TSQuaternion result;
            Add(ref value1, ref value2, out result);
            return result;
        }

        #endregion

        /// <summary>
        /// Subtract two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The difference of both quaternions.</returns>

        #region public static FP operator -(JQuaternion value1, JQuaternion value2)

        public static TSQuaternion operator -(TSQuaternion value1, TSQuaternion value2) {
            TSQuaternion result;
            Subtract(ref value1, ref value2, out result);
            return result;
        }

        #endregion

        /**
         *  @brief Rotates a {@link TSVector} by the {@link TSQuanternion}.
         **/
        public static TSVector operator *(TSQuaternion quat, TSVector vec) {
            var num = quat.x * 2;
            var num2 = quat.y * 2;
            var num3 = quat.z * 2;
            var num4 = quat.x * num;
            var num5 = quat.y * num2;
            var num6 = quat.z * num3;
            var num7 = quat.x * num2;
            var num8 = quat.x * num3;
            var num9 = quat.y * num3;
            var num10 = quat.w * num;
            var num11 = quat.w * num2;
            var num12 = quat.w * num3;

            TSVector result;
            result.x = (FixedPoint.One - (num5 + num6)) * vec.x + (num7 - num12) * vec.y + (num8 + num11) * vec.z;
            result.y = (num7 + num12) * vec.x + (FixedPoint.One - (num4 + num6)) * vec.y + (num9 - num10) * vec.z;
            result.z = (num8 - num11) * vec.x + (num9 + num10) * vec.y + (FixedPoint.One - (num4 + num5)) * vec.z;

            return result;
        }

        public override string ToString() {
            return string.Format("({0:f1}, {1:f1}, {2:f1}, {3:f1})", x.AsFloat(), y.AsFloat(), z.AsFloat(),
                w.AsFloat());
        }

        public static TSQuaternion Left90Rotate = Euler(0, -90, 0);
        public static TSQuaternion Right90Rotate = Euler(0, 90, 0);
    }
}