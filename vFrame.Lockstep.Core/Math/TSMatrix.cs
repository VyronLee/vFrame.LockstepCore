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

namespace vFrame.Lockstep.Core
{

    /// <summary>
    /// 3x3 Matrix.
    /// </summary>
    public struct TSMatrix
    {
        /// <summary>
        /// M11
        /// </summary>
        public FixedPoint M11; // 1st row vector
        /// <summary>
        /// M12
        /// </summary>
        public FixedPoint M12;
        /// <summary>
        /// M13
        /// </summary>
        public FixedPoint M13;
        /// <summary>
        /// M21
        /// </summary>
        public FixedPoint M21; // 2nd row vector
        /// <summary>
        /// M22
        /// </summary>
        public FixedPoint M22;
        /// <summary>
        /// M23
        /// </summary>
        public FixedPoint M23;
        /// <summary>
        /// M31
        /// </summary>
        public FixedPoint M31; // 3rd row vector
        /// <summary>
        /// M32
        /// </summary>
        public FixedPoint M32;
        /// <summary>
        /// M33
        /// </summary>
        public FixedPoint M33;

        internal static TSMatrix InternalIdentity;

        /// <summary>
        /// Identity matrix.
        /// </summary>
        public static readonly TSMatrix Identity;
        public static readonly TSMatrix Zero;

        static TSMatrix()
        {
            Zero = new TSMatrix();

            Identity = new TSMatrix();
            Identity.M11 = FixedPoint.One;
            Identity.M22 = FixedPoint.One;
            Identity.M33 = FixedPoint.One;

            InternalIdentity = Identity;
        }

        public TSVector eulerAngles {
            get {
                TSVector result = new TSVector();

                result.x = TSMath.Atan2(M32, M33) * FixedPoint.Rad2Deg;
                result.y = TSMath.Atan2(-M31, TSMath.Sqrt(M32 * M32 + M33 * M33)) * FixedPoint.Rad2Deg;
                result.z = TSMath.Atan2(M21, M11) * FixedPoint.Rad2Deg;

                return result * -1;
            }
        }

        public static TSMatrix CreateFromYawPitchRoll(FixedPoint yaw, FixedPoint pitch, FixedPoint roll)
        {
            TSMatrix matrix;
            TSQuaternion quaternion;
            TSQuaternion.CreateFromYawPitchRoll(yaw, pitch, roll, out quaternion);
            CreateFromQuaternion(ref quaternion, out matrix);
            return matrix;
        }

        public static TSMatrix CreateRotationX(FixedPoint radians)
        {
            TSMatrix matrix;
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            matrix.M11 = FixedPoint.One;
            matrix.M12 = FixedPoint.Zero;
            matrix.M13 = FixedPoint.Zero;
            matrix.M21 = FixedPoint.Zero;
            matrix.M22 = num2;
            matrix.M23 = num;
            matrix.M31 = FixedPoint.Zero;
            matrix.M32 = -num;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationX(FixedPoint radians, out TSMatrix result)
        {
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            result.M11 = FixedPoint.One;
            result.M12 = FixedPoint.Zero;
            result.M13 = FixedPoint.Zero;
            result.M21 = FixedPoint.Zero;
            result.M22 = num2;
            result.M23 = num;
            result.M31 = FixedPoint.Zero;
            result.M32 = -num;
            result.M33 = num2;
        }

        public static TSMatrix CreateRotationY(FixedPoint radians)
        {
            TSMatrix matrix;
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            matrix.M11 = num2;
            matrix.M12 = FixedPoint.Zero;
            matrix.M13 = -num;
            matrix.M21 = FixedPoint.Zero;
            matrix.M22 = FixedPoint.One;
            matrix.M23 = FixedPoint.Zero;
            matrix.M31 = num;
            matrix.M32 = FixedPoint.Zero;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationY(FixedPoint radians, out TSMatrix result)
        {
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            result.M11 = num2;
            result.M12 = FixedPoint.Zero;
            result.M13 = -num;
            result.M21 = FixedPoint.Zero;
            result.M22 = FixedPoint.One;
            result.M23 = FixedPoint.Zero;
            result.M31 = num;
            result.M32 = FixedPoint.Zero;
            result.M33 = num2;
        }

        public static TSMatrix CreateRotationZ(FixedPoint radians)
        {
            TSMatrix matrix;
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            matrix.M11 = num2;
            matrix.M12 = num;
            matrix.M13 = FixedPoint.Zero;
            matrix.M21 = -num;
            matrix.M22 = num2;
            matrix.M23 = FixedPoint.Zero;
            matrix.M31 = FixedPoint.Zero;
            matrix.M32 = FixedPoint.Zero;
            matrix.M33 = FixedPoint.One;
            return matrix;
        }


        public static void CreateRotationZ(FixedPoint radians, out TSMatrix result)
        {
            FixedPoint num2 = FixedPoint.Cos(radians);
            FixedPoint num = FixedPoint.Sin(radians);
            result.M11 = num2;
            result.M12 = num;
            result.M13 = FixedPoint.Zero;
            result.M21 = -num;
            result.M22 = num2;
            result.M23 = FixedPoint.Zero;
            result.M31 = FixedPoint.Zero;
            result.M32 = FixedPoint.Zero;
            result.M33 = FixedPoint.One;
        }

        /// <summary>
        /// Initializes a new instance of the matrix structure.
        /// </summary>
        /// <param name="m11">m11</param>
        /// <param name="m12">m12</param>
        /// <param name="m13">m13</param>
        /// <param name="m21">m21</param>
        /// <param name="m22">m22</param>
        /// <param name="m23">m23</param>
        /// <param name="m31">m31</param>
        /// <param name="m32">m32</param>
        /// <param name="m33">m33</param>
        #region public JMatrix(FP m11, FP m12, FP m13, FP m21, FP m22, FP m23,FP m31, FP m32, FP m33)
        public TSMatrix(FixedPoint m11, FixedPoint m12, FixedPoint m13, FixedPoint m21, FixedPoint m22, FixedPoint m23,FixedPoint m31, FixedPoint m32, FixedPoint m33)
        {
            this.M11 = m11;
            this.M12 = m12;
            this.M13 = m13;
            this.M21 = m21;
            this.M22 = m22;
            this.M23 = m23;
            this.M31 = m31;
            this.M32 = m32;
            this.M33 = m33;
        }
        #endregion

        /// <summary>
        /// Gets the determinant of the matrix.
        /// </summary>
        /// <returns>The determinant of the matrix.</returns>
        #region public FP Determinant()
        //public FP Determinant()
        //{
        //    return M11 * M22 * M33 -M11 * M23 * M32 -M12 * M21 * M33 +M12 * M23 * M31 + M13 * M21 * M32 - M13 * M22 * M31;
        //}
        #endregion

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The product of both matrices.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        public static TSMatrix Multiply(TSMatrix matrix1, TSMatrix matrix2)
        {
            TSMatrix result;
            TSMatrix.Multiply(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The product of both matrices.</param>
        public static void Multiply(ref TSMatrix matrix1, ref TSMatrix matrix2, out TSMatrix result)
        {
            FixedPoint num0 = ((matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21)) + (matrix1.M13 * matrix2.M31);
            FixedPoint num1 = ((matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22)) + (matrix1.M13 * matrix2.M32);
            FixedPoint num2 = ((matrix1.M11 * matrix2.M13) + (matrix1.M12 * matrix2.M23)) + (matrix1.M13 * matrix2.M33);
            FixedPoint num3 = ((matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21)) + (matrix1.M23 * matrix2.M31);
            FixedPoint num4 = ((matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22)) + (matrix1.M23 * matrix2.M32);
            FixedPoint num5 = ((matrix1.M21 * matrix2.M13) + (matrix1.M22 * matrix2.M23)) + (matrix1.M23 * matrix2.M33);
            FixedPoint num6 = ((matrix1.M31 * matrix2.M11) + (matrix1.M32 * matrix2.M21)) + (matrix1.M33 * matrix2.M31);
            FixedPoint num7 = ((matrix1.M31 * matrix2.M12) + (matrix1.M32 * matrix2.M22)) + (matrix1.M33 * matrix2.M32);
            FixedPoint num8 = ((matrix1.M31 * matrix2.M13) + (matrix1.M32 * matrix2.M23)) + (matrix1.M33 * matrix2.M33);

            result.M11 = num0;
            result.M12 = num1;
            result.M13 = num2;
            result.M21 = num3;
            result.M22 = num4;
            result.M23 = num5;
            result.M31 = num6;
            result.M32 = num7;
            result.M33 = num8;
        }
        #endregion

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The sum of both matrices.</returns>
        #region public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        public static TSMatrix Add(TSMatrix matrix1, TSMatrix matrix2)
        {
            TSMatrix result;
            TSMatrix.Add(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The sum of both matrices.</param>
        public static void Add(ref TSMatrix matrix1, ref TSMatrix matrix2, out TSMatrix result)
        {
            result.M11 = matrix1.M11 + matrix2.M11;
            result.M12 = matrix1.M12 + matrix2.M12;
            result.M13 = matrix1.M13 + matrix2.M13;
            result.M21 = matrix1.M21 + matrix2.M21;
            result.M22 = matrix1.M22 + matrix2.M22;
            result.M23 = matrix1.M23 + matrix2.M23;
            result.M31 = matrix1.M31 + matrix2.M31;
            result.M32 = matrix1.M32 + matrix2.M32;
            result.M33 = matrix1.M33 + matrix2.M33;
        }
        #endregion

        /// <summary>
        /// Calculates the inverse of a give matrix.
        /// </summary>
        /// <param name="matrix">The matrix to invert.</param>
        /// <returns>The inverted JMatrix.</returns>
        #region public static JMatrix Inverse(JMatrix matrix)
        public static TSMatrix Inverse(TSMatrix matrix)
        {
            TSMatrix result;
            TSMatrix.Inverse(ref matrix, out result);
            return result;
        }

        public FixedPoint Determinant()
        {
            return M11 * M22 * M33 + M12 * M23 * M31 + M13 * M21 * M32 -
                   M31 * M22 * M13 - M32 * M23 * M11 - M33 * M21 * M12;
        }

        public static void Invert(ref TSMatrix matrix, out TSMatrix result)
        {
            FixedPoint determinantInverse = 1 / matrix.Determinant();
            FixedPoint m11 = (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * determinantInverse;
            FixedPoint m12 = (matrix.M13 * matrix.M32 - matrix.M33 * matrix.M12) * determinantInverse;
            FixedPoint m13 = (matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13) * determinantInverse;

            FixedPoint m21 = (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33) * determinantInverse;
            FixedPoint m22 = (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31) * determinantInverse;
            FixedPoint m23 = (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23) * determinantInverse;

            FixedPoint m31 = (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31) * determinantInverse;
            FixedPoint m32 = (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32) * determinantInverse;
            FixedPoint m33 = (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21) * determinantInverse;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;

            result.M31 = m31;
            result.M32 = m32;
            result.M33 = m33;
        }

        /// <summary>
        /// Calculates the inverse of a give matrix.
        /// </summary>
        /// <param name="matrix">The matrix to invert.</param>
        /// <param name="result">The inverted JMatrix.</param>
        public static void Inverse(ref TSMatrix matrix, out TSMatrix result)
        {
			FixedPoint det = 1024 * matrix.M11 * matrix.M22 * matrix.M33 -
				1024 * matrix.M11 * matrix.M23 * matrix.M32 -
				1024 * matrix.M12 * matrix.M21 * matrix.M33 +
				1024 * matrix.M12 * matrix.M23 * matrix.M31 +
				1024 * matrix.M13 * matrix.M21 * matrix.M32 -
				1024 * matrix.M13 * matrix.M22 * matrix.M31;

			FixedPoint num11 =1024* matrix.M22 * matrix.M33 - 1024*matrix.M23 * matrix.M32;
			FixedPoint num12 =1024* matrix.M13 * matrix.M32 -1024* matrix.M12 * matrix.M33;
			FixedPoint num13 =1024* matrix.M12 * matrix.M23 -1024* matrix.M22 * matrix.M13;

			FixedPoint num21 =1024* matrix.M23 * matrix.M31 -1024* matrix.M33 * matrix.M21;
			FixedPoint num22 =1024* matrix.M11 * matrix.M33 -1024* matrix.M31 * matrix.M13;
			FixedPoint num23 =1024* matrix.M13 * matrix.M21 -1024* matrix.M23 * matrix.M11;

			FixedPoint num31 =1024* matrix.M21 * matrix.M32 - 1024* matrix.M31 * matrix.M22;
			FixedPoint num32 =1024* matrix.M12 * matrix.M31 - 1024* matrix.M32 * matrix.M11;
			FixedPoint num33 =1024* matrix.M11 * matrix.M22 - 1024*matrix.M21 * matrix.M12;

			if(det == 0){
				result.M11 = FixedPoint.PositiveInfinity;
				result.M12 = FixedPoint.PositiveInfinity;
				result.M13 = FixedPoint.PositiveInfinity;
				result.M21 = FixedPoint.PositiveInfinity;
				result.M22 = FixedPoint.PositiveInfinity;
				result.M23 = FixedPoint.PositiveInfinity;
				result.M31 = FixedPoint.PositiveInfinity;
				result.M32 = FixedPoint.PositiveInfinity;
				result.M33 = FixedPoint.PositiveInfinity;
			} else{
				result.M11 = num11 / det;
				result.M12 = num12 / det;
				result.M13 = num13 / det;
				result.M21 = num21 / det;
				result.M22 = num22 / det;
				result.M23 = num23 / det;
				result.M31 = num31 / det;
				result.M32 = num32 / det;
				result.M33 = num33 / det;
			}
            
        }
        #endregion

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <returns>A JMatrix multiplied by the scale factor.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, FP scaleFactor)
        public static TSMatrix Multiply(TSMatrix matrix1, FixedPoint scaleFactor)
        {
            TSMatrix result;
            TSMatrix.Multiply(ref matrix1, scaleFactor, out result);
            return result;
        }

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <param name="result">A JMatrix multiplied by the scale factor.</param>
        public static void Multiply(ref TSMatrix matrix1, FixedPoint scaleFactor, out TSMatrix result)
        {
            FixedPoint num = scaleFactor;
            result.M11 = matrix1.M11 * num;
            result.M12 = matrix1.M12 * num;
            result.M13 = matrix1.M13 * num;
            result.M21 = matrix1.M21 * num;
            result.M22 = matrix1.M22 * num;
            result.M23 = matrix1.M23 * num;
            result.M31 = matrix1.M31 * num;
            result.M32 = matrix1.M32 * num;
            result.M33 = matrix1.M33 * num;
        }
        #endregion

        /// <summary>
        /// Creates a JMatrix representing an orientation from a quaternion.
        /// </summary>
        /// <param name="quaternion">The quaternion the matrix should be created from.</param>
        /// <returns>JMatrix representing an orientation.</returns>
        #region public static JMatrix CreateFromQuaternion(JQuaternion quaternion)

		public static TSMatrix CreateFromLookAt(TSVector position, TSVector target){
			TSMatrix result;
			LookAt (target - position, TSVector.up, out result);
			return result;
		}

        public static TSMatrix LookAt(TSVector forward, TSVector upwards) {
            TSMatrix result;
            LookAt(forward, upwards, out result);

            return result;
        }

        public static void LookAt(TSVector forward, TSVector upwards, out TSMatrix result) {
            TSVector zaxis = forward; zaxis.Normalize();
            TSVector xaxis = TSVector.Cross(upwards, zaxis); xaxis.Normalize();
            TSVector yaxis = TSVector.Cross(zaxis, xaxis);

            result.M11 = xaxis.x;
            result.M21 = yaxis.x;
            result.M31 = zaxis.x;
            result.M12 = xaxis.y;
            result.M22 = yaxis.y;
            result.M32 = zaxis.y;
            result.M13 = xaxis.z;
            result.M23 = yaxis.z;
            result.M33 = zaxis.z;
        }

        public static TSMatrix CreateFromQuaternion(TSQuaternion quaternion)
        {
            TSMatrix result;
            TSMatrix.CreateFromQuaternion(ref quaternion,out result);
            return result;
        }

        /// <summary>
        /// Creates a JMatrix representing an orientation from a quaternion.
        /// </summary>
        /// <param name="quaternion">The quaternion the matrix should be created from.</param>
        /// <param name="result">JMatrix representing an orientation.</param>
        public static void CreateFromQuaternion(ref TSQuaternion quaternion, out TSMatrix result)
        {
            FixedPoint num9 = quaternion.x * quaternion.x;
            FixedPoint num8 = quaternion.y * quaternion.y;
            FixedPoint num7 = quaternion.z * quaternion.z;
            FixedPoint num6 = quaternion.x * quaternion.y;
            FixedPoint num5 = quaternion.z * quaternion.w;
            FixedPoint num4 = quaternion.z * quaternion.x;
            FixedPoint num3 = quaternion.y * quaternion.w;
            FixedPoint num2 = quaternion.y * quaternion.z;
            FixedPoint num = quaternion.x * quaternion.w;
            result.M11 = FixedPoint.One - (2 * (num8 + num7));
            result.M12 = 2 * (num6 + num5);
            result.M13 = 2 * (num4 - num3);
            result.M21 = 2 * (num6 - num5);
            result.M22 = FixedPoint.One - (2 * (num7 + num9));
            result.M23 = 2 * (num2 + num);
            result.M31 = 2 * (num4 + num3);
            result.M32 = 2 * (num2 - num);
            result.M33 = FixedPoint.One - (2 * (num8 + num9));
        }
        #endregion

        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <returns>The transposed JMatrix.</returns>
        #region public static JMatrix Transpose(JMatrix matrix)
        public static TSMatrix Transpose(TSMatrix matrix)
        {
            TSMatrix result;
            TSMatrix.Transpose(ref matrix, out result);
            return result;
        }

        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <param name="result">The transposed JMatrix.</param>
        public static void Transpose(ref TSMatrix matrix, out TSMatrix result)
        {
            result.M11 = matrix.M11;
            result.M12 = matrix.M21;
            result.M13 = matrix.M31;
            result.M21 = matrix.M12;
            result.M22 = matrix.M22;
            result.M23 = matrix.M32;
            result.M31 = matrix.M13;
            result.M32 = matrix.M23;
            result.M33 = matrix.M33;
        }
        #endregion

        /// <summary>
        /// Multiplies two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The product of both values.</returns>
        #region public static JMatrix operator *(JMatrix value1,JMatrix value2)
        public static TSMatrix operator *(TSMatrix value1,TSMatrix value2)
        {
            TSMatrix result; TSMatrix.Multiply(ref value1, ref value2, out result);
            return result;
        }
        #endregion


        public FixedPoint Trace()
        {
            return this.M11 + this.M22 + this.M33;
        }

        /// <summary>
        /// Adds two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The sum of both values.</returns>
        #region public static JMatrix operator +(JMatrix value1, JMatrix value2)
        public static TSMatrix operator +(TSMatrix value1, TSMatrix value2)
        {
            TSMatrix result; TSMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        /// <summary>
        /// Subtracts two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The difference of both values.</returns>
        #region public static JMatrix operator -(JMatrix value1, JMatrix value2)
        public static TSMatrix operator -(TSMatrix value1, TSMatrix value2)
        {
            TSMatrix result; TSMatrix.Multiply(ref value2, -FixedPoint.One, out value2);
            TSMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        public static bool operator == (TSMatrix value1, TSMatrix value2) {
            return value1.M11 == value2.M11 &&
                value1.M12 == value2.M12 &&
                value1.M13 == value2.M13 &&
                value1.M21 == value2.M21 &&
                value1.M22 == value2.M22 &&
                value1.M23 == value2.M23 &&
                value1.M31 == value2.M31 &&
                value1.M32 == value2.M32 &&
                value1.M33 == value2.M33;
        }

        public static bool operator != (TSMatrix value1, TSMatrix value2) {
            return value1.M11 != value2.M11 ||
                value1.M12 != value2.M12 ||
                value1.M13 != value2.M13 ||
                value1.M21 != value2.M21 ||
                value1.M22 != value2.M22 ||
                value1.M23 != value2.M23 ||
                value1.M31 != value2.M31 ||
                value1.M32 != value2.M32 ||
                value1.M33 != value2.M33;
        }

        public override bool Equals(object obj) {
            if (!(obj is TSMatrix)) return false;
            TSMatrix other = (TSMatrix) obj;

            return this.M11 == other.M11 &&
                this.M12 == other.M12 &&
                this.M13 == other.M13 &&
                this.M21 == other.M21 &&
                this.M22 == other.M22 &&
                this.M23 == other.M23 &&
                this.M31 == other.M31 &&
                this.M32 == other.M32 &&
                this.M33 == other.M33;
        }

        public override int GetHashCode() {
            return M11.GetHashCode() ^
                M12.GetHashCode() ^
                M13.GetHashCode() ^
                M21.GetHashCode() ^
                M22.GetHashCode() ^
                M23.GetHashCode() ^
                M31.GetHashCode() ^
                M32.GetHashCode() ^
                M33.GetHashCode();
        }

        /// <summary>
        /// Creates a matrix which rotates around the given axis by the given angle.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="angle">The angle.</param>
        /// <param name="result">The resulting rotation matrix</param>
        #region public static void CreateFromAxisAngle(ref JVector axis, FP angle, out JMatrix result)
        public static void CreateFromAxisAngle(ref TSVector axis, FixedPoint angle, out TSMatrix result)
        {
            FixedPoint x = axis.x;
            FixedPoint y = axis.y;
            FixedPoint z = axis.z;
            FixedPoint num2 = FixedPoint.Sin(angle);
            FixedPoint num = FixedPoint.Cos(angle);
            FixedPoint num11 = x * x;
            FixedPoint num10 = y * y;
            FixedPoint num9 = z * z;
            FixedPoint num8 = x * y;
            FixedPoint num7 = x * z;
            FixedPoint num6 = y * z;
            result.M11 = num11 + (num * (FixedPoint.One - num11));
            result.M12 = (num8 - (num * num8)) + (num2 * z);
            result.M13 = (num7 - (num * num7)) - (num2 * y);
            result.M21 = (num8 - (num * num8)) - (num2 * z);
            result.M22 = num10 + (num * (FixedPoint.One - num10));
            result.M23 = (num6 - (num * num6)) + (num2 * x);
            result.M31 = (num7 - (num * num7)) + (num2 * y);
            result.M32 = (num6 - (num * num6)) - (num2 * x);
            result.M33 = num9 + (num * (FixedPoint.One - num9));
        }

        /// <summary>
        /// Creates a matrix which rotates around the given axis by the given angle.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="angle">The angle.</param>
        /// <returns>The resulting rotation matrix</returns>
        public static TSMatrix AngleAxis(FixedPoint angle, TSVector axis)
        {
            TSMatrix result; CreateFromAxisAngle(ref axis, angle, out result);
            return result;
        }

        #endregion

        public override string ToString() {
            return string.Format("{0}|{1}|{2}|{3}|{4}|{5}|{6}|{7}|{8}", M11.RawValue, M12.RawValue, M13.RawValue, M21.RawValue, M22.RawValue, M23.RawValue, M31.RawValue, M32.RawValue, M33.RawValue);
        }

    }

}