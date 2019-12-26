using System;

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
    /// Contains common math operations.
    /// </summary>
    public sealed class TSMath
    {
        /// <summary>
        /// PI constant.
        /// </summary>
        public static FixedPoint Pi = FixedPoint.Pi;

        /**
        *  @brief PI over 2 constant.
        **/
        public static FixedPoint PiOver2 = FixedPoint.PiOver2;

        /// <summary>
        /// A small value often used to decide if numeric
        /// results are zero.
        /// </summary>
        public static FixedPoint Epsilon = FixedPoint.Epsilon;

        /**
        *  @brief Degree to radians constant.
        **/
        public static FixedPoint Deg2Rad = FixedPoint.Deg2Rad;

        /**
        *  @brief Radians to degree constant.
        **/
        public static FixedPoint Rad2Deg = FixedPoint.Rad2Deg;


        /**
         * @brief FP infinity.
         * */
        public static FixedPoint Infinity = FixedPoint.MaxValue;

        /// <summary>
        /// Gets the square root.
        /// </summary>
        /// <param name="number">The number to get the square root from.</param>
        /// <returns></returns>

        #region public static FP Sqrt(FP number)

        public static FixedPoint Sqrt(FixedPoint number) {
            return FixedPoint.Sqrt(number);
        }

        #endregion

        /// <summary>
        /// Gets the maximum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the largest value.</returns>

        #region public static FP Max(FP val1, FP val2)

        public static FixedPoint Max(FixedPoint val1, FixedPoint val2) {
            return (val1 > val2) ? val1 : val2;
        }

        #endregion

        /// <summary>
        /// Gets the minimum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the smallest value.</returns>

        #region public static FP Min(FP val1, FP val2)

        public static FixedPoint Min(FixedPoint val1, FixedPoint val2) {
            return (val1 < val2) ? val1 : val2;
        }

        #endregion

        /// <summary>
        /// Gets the maximum number of three values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <param name="val3">The third value.</param>
        /// <returns>Returns the largest value.</returns>

        #region public static FP Max(FP val1, FP val2,FP val3)

        public static FixedPoint Max(FixedPoint val1, FixedPoint val2, FixedPoint val3) {
            FixedPoint max12 = (val1 > val2) ? val1 : val2;
            return (max12 > val3) ? max12 : val3;
        }

        #endregion

        /// <summary>
        /// Returns a number which is within [min,max]
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <param name="min">The minimum value.</param>
        /// <param name="max">The maximum value.</param>
        /// <returns>The clamped value.</returns>

        #region public static FP Clamp(FP value, FP min, FP max)

        public static FixedPoint Clamp(FixedPoint value, FixedPoint min, FixedPoint max) {
            if (value < min) {
                value = min;
                return value;
            }

            if (value > max) {
                value = max;
            }

            return value;
        }

        #endregion

        /// <summary>
        /// Returns a number which is within [FP.Zero, FP.One]
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <returns>The clamped value.</returns>
        public static FixedPoint Clamp01(FixedPoint value) {
            if (value < FixedPoint.Zero)
                return FixedPoint.Zero;

            if (value > FixedPoint.One)
                return FixedPoint.One;

            return value;
        }

        /// <summary>
        /// Changes every sign of the matrix entry to '+'
        /// </summary>
        /// <param name="matrix">The matrix.</param>
        /// <param name="result">The absolute matrix.</param>

        #region public static void Absolute(ref JMatrix matrix,out JMatrix result)

        public static void Absolute(ref TSMatrix matrix, out TSMatrix result) {
            result.M11 = FixedPoint.Abs(matrix.M11);
            result.M12 = FixedPoint.Abs(matrix.M12);
            result.M13 = FixedPoint.Abs(matrix.M13);
            result.M21 = FixedPoint.Abs(matrix.M21);
            result.M22 = FixedPoint.Abs(matrix.M22);
            result.M23 = FixedPoint.Abs(matrix.M23);
            result.M31 = FixedPoint.Abs(matrix.M31);
            result.M32 = FixedPoint.Abs(matrix.M32);
            result.M33 = FixedPoint.Abs(matrix.M33);
        }

        #endregion

        /// <summary>
        /// Returns the sine of value.
        /// </summary>
        public static FixedPoint Sin(FixedPoint value) {
            return FixedPoint.FastSin(value);
        }

        /// <summary>
        /// Returns the cosine of value.
        /// </summary>
        public static FixedPoint Cos(FixedPoint value) {
            return FixedPoint.FastCos(value);
        }

        /// <summary>
        /// Returns the tan of value.
        /// </summary>
        public static FixedPoint Tan(FixedPoint value) {
            return FixedPoint.Tan(value);
        }

        /// <summary>
        /// Returns the arc sine of value.
        /// </summary>
        public static FixedPoint Asin(FixedPoint value) {
            return FixedPoint.Asin(value);
        }

        /// <summary>
        /// Returns the arc cosine of value.
        /// </summary>
        public static FixedPoint Acos(FixedPoint value) {
            return FixedPoint.Acos(value);
        }

        /// <summary>
        /// Returns the arc tan of value.
        /// </summary>
        public static FixedPoint Atan(FixedPoint value) {
            return FixedPoint.Atan(value);
        }

        /// <summary>
        /// Returns the arc tan of coordinates x-y.
        /// </summary>
        public static FixedPoint Atan2(FixedPoint y, FixedPoint x) {
            return FixedPoint.Atan2(y, x);
        }

        /// <summary>
        /// Returns the largest integer less than or equal to the specified number.
        /// </summary>
        public static FixedPoint Floor(FixedPoint value) {
            return FixedPoint.Floor(value);
        }

        /// <summary>
        /// Returns the smallest integral value that is greater than or equal to the specified number.
        /// </summary>
        public static FixedPoint Ceiling(FixedPoint value) {
            return FixedPoint.Ceiling(value);
        }

        /// <summary>
        /// Rounds a value to the nearest integral value.
        /// If the value is halfway between an even and an uneven value, returns the even value.
        /// </summary>
        public static FixedPoint Round(FixedPoint value) {
            return FixedPoint.Round(value);
        }

        /// <summary>
        /// Returns a number indicating the sign of a Fix64 number.
        /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
        /// </summary>
        public static int Sign(FixedPoint value) {
            return FixedPoint.Sign(value);
        }

        /// <summary>
        /// Returns the absolute value of a Fix64 number.
        /// Note: Abs(Fix64.MinValue) == Fix64.MaxValue.
        /// </summary>
        public static FixedPoint Abs(FixedPoint value) {
            return FixedPoint.Abs(value);
        }

        public static FixedPoint Barycentric(FixedPoint value1, FixedPoint value2, FixedPoint value3,
            FixedPoint amount1, FixedPoint amount2) {
            return value1 + (value2 - value1) * amount1 + (value3 - value1) * amount2;
        }

        public static FixedPoint CatmullRom(FixedPoint value1, FixedPoint value2, FixedPoint value3, FixedPoint value4,
            FixedPoint amount) {
            // Using formula from http://www.mvps.org/directx/articles/catmull/
            // Internally using FPs not to lose precission
            FixedPoint amountSquared = amount * amount;
            FixedPoint amountCubed = amountSquared * amount;
            return (FixedPoint) (FixedPoint.Half * (2 * value2 +
                                                    (value3 - value1) * amount +
                                                    (2 * value1 - 5 * value2 + 4 * value3 - value4) * amountSquared +
                                                    (3 * value2 - value1 - 3 * value3 + value4) * amountCubed));
        }

        public static FixedPoint Distance(FixedPoint value1, FixedPoint value2) {
            return FixedPoint.Abs(value1 - value2);
        }

        public static FixedPoint Hermite(FixedPoint value1, FixedPoint tangent1, FixedPoint value2, FixedPoint tangent2,
            FixedPoint amount) {
            // All transformed to FP not to lose precission
            // Otherwise, for high numbers of param:amount the result is NaN instead of Infinity
            FixedPoint v1 = value1, v2 = value2, t1 = tangent1, t2 = tangent2, s = amount, result;
            FixedPoint sCubed = s * s * s;
            FixedPoint sSquared = s * s;

            if (amount == FixedPoint.Zero)
                result = value1;
            else if (amount == FixedPoint.One)
                result = value2;
            else
                result = (2 * v1 - 2 * v2 + t2 + t1) * sCubed +
                         (3 * v2 - 3 * v1 - 2 * t1 - t2) * sSquared +
                         t1 * s +
                         v1;
            return (FixedPoint) result;
        }

        public static FixedPoint Lerp(FixedPoint value1, FixedPoint value2, FixedPoint amount) {
            return value1 + (value2 - value1) * Clamp01(amount);
        }

        public static FixedPoint InverseLerp(FixedPoint value1, FixedPoint value2, FixedPoint amount) {
            if (value1 != value2)
                return Clamp01((amount - value1) / (value2 - value1));
            return FixedPoint.Zero;
        }

        public static FixedPoint SmoothStep(FixedPoint value1, FixedPoint value2, FixedPoint amount) {
            // It is expected that 0 < amount < 1
            // If amount < 0, return value1
            // If amount > 1, return value2
            FixedPoint result = Clamp(amount, FixedPoint.Zero, FixedPoint.One);
            result = Hermite(value1, FixedPoint.Zero, value2, FixedPoint.Zero, result);
            return result;
        }


        /// <summary>
        /// Returns 2 raised to the specified power.
        /// Provides at least 6 decimals of accuracy.
        /// </summary>
        internal static FixedPoint Pow2(FixedPoint x) {
            if (x.RawValue == 0) {
                return FixedPoint.One;
            }

            // Avoid negative arguments by exploiting that exp(-x) = 1/exp(x).
            bool neg = x.RawValue < 0;
            if (neg) {
                x = -x;
            }

            if (x == FixedPoint.One) {
                return neg ? FixedPoint.One / (FixedPoint) 2 : (FixedPoint) 2;
            }

            if (x >= FixedPoint.Log2Max) {
                return neg ? FixedPoint.One / FixedPoint.MaxValue : FixedPoint.MaxValue;
            }

            if (x <= FixedPoint.Log2Min) {
                return neg ? FixedPoint.MaxValue : FixedPoint.Zero;
            }

            /* The algorithm is based on the power series for exp(x):
             * http://en.wikipedia.org/wiki/Exponential_function#Formal_definition
             *
             * From term n, we get term n+1 by multiplying with x/n.
             * When the sum term drops to zero, we can stop summing.
             */

            int integerPart = (int) Floor(x);
            // Take fractional part of exponent
            x = FixedPoint.FromRaw(x.RawValue & 0x00000000FFFFFFFF);

            var result = FixedPoint.One;
            var term = FixedPoint.One;
            int i = 1;
            while (term.RawValue != 0) {
                term = FixedPoint.FastMul(FixedPoint.FastMul(x, term), FixedPoint.Ln2) / (FixedPoint) i;
                result += term;
                i++;
            }

            result = FixedPoint.FromRaw(result.RawValue << integerPart);
            if (neg) {
                result = FixedPoint.One / result;
            }

            return result;
        }

        /// <summary>
        /// Returns the base-2 logarithm of a specified number.
        /// Provides at least 9 decimals of accuracy.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The argument was non-positive
        /// </exception>
        internal static FixedPoint Log2(FixedPoint x) {
            if (x.RawValue <= 0) {
                throw new ArgumentOutOfRangeException("Non-positive value passed to Ln", "x");
            }

            // This implementation is based on Clay. S. Turner's fast binary logarithm
            // algorithm (C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal
            //     Processing Mag., pp. 124,140, Sep. 2010.)

            long b = 1U << (FixedPoint.FRACTIONAL_PLACES - 1);
            long y = 0;

            long rawX = x.RawValue;
            while (rawX < FixedPoint.L_ONE) {
                rawX <<= 1;
                y -= FixedPoint.L_ONE;
            }

            while (rawX >= (FixedPoint.L_ONE << 1)) {
                rawX >>= 1;
                y += FixedPoint.L_ONE;
            }

            var z = FixedPoint.FromRaw(rawX);

            for (int i = 0; i < FixedPoint.FRACTIONAL_PLACES; i++) {
                z = FixedPoint.FastMul(z, z);
                if (z.RawValue >= (FixedPoint.L_ONE << 1)) {
                    z = FixedPoint.FromRaw(z.RawValue >> 1);
                    y += b;
                }

                b >>= 1;
            }

            return FixedPoint.FromRaw(y);
        }

        /// <summary>
        /// Returns the natural logarithm of a specified number.
        /// Provides at least 7 decimals of accuracy.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The argument was non-positive
        /// </exception>
        public static FixedPoint Ln(FixedPoint x) {
            return FixedPoint.FastMul(Log2(x), FixedPoint.Ln2);
        }

        /// <summary>
        /// Returns a specified number raised to the specified power.
        /// Provides about 5 digits of accuracy for the result.
        /// </summary>
        /// <exception cref="DivideByZeroException">
        /// The base was zero, with a negative exponent
        /// </exception>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The base was negative, with a non-zero exponent
        /// </exception>
        public static FixedPoint Pow(FixedPoint b, FixedPoint exp) {
            if (b == FixedPoint.One) {
                return FixedPoint.One;
            }

            if (exp.RawValue == 0) {
                return FixedPoint.One;
            }

            if (b.RawValue == 0) {
                if (exp.RawValue < 0) {
                    //throw new DivideByZeroException();
                    return FixedPoint.MaxValue;
                }

                return FixedPoint.Zero;
            }

            FixedPoint log2 = Log2(b);
            return Pow2(exp * log2);
        }

        public static FixedPoint MoveTowards(FixedPoint current, FixedPoint target, FixedPoint maxDelta) {
            if (Abs(target - current) <= maxDelta)
                return target;
            return (current + (Sign(target - current)) * maxDelta);
        }

        public static FixedPoint Repeat(FixedPoint t, FixedPoint length) {
            return (t - (Floor(t / length) * length));
        }

        public static FixedPoint DeltaAngle(FixedPoint current, FixedPoint target) {
            FixedPoint num = Repeat(target - current, (FixedPoint) 360f);
            if (num > (FixedPoint) 180f) {
                num -= (FixedPoint) 360f;
            }

            return num;
        }

        public static FixedPoint MoveTowardsAngle(FixedPoint current, FixedPoint target, FixedPoint maxDelta) {
            target = current + DeltaAngle(current, target);
            return MoveTowards(current, target, maxDelta);
        }

        public static FixedPoint SmoothDamp(FixedPoint current, FixedPoint target, ref FixedPoint currentVelocity,
            FixedPoint smoothTime, FixedPoint maxSpeed) {
            FixedPoint deltaTime = FixedPoint.EN2;
            return SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime);
        }

        public static FixedPoint SmoothDamp(FixedPoint current, FixedPoint target, ref FixedPoint currentVelocity,
            FixedPoint smoothTime) {
            FixedPoint deltaTime = FixedPoint.EN2;
            FixedPoint positiveInfinity = -FixedPoint.MaxValue;
            return SmoothDamp(current, target, ref currentVelocity, smoothTime, positiveInfinity, deltaTime);
        }

        public static FixedPoint SmoothDamp(FixedPoint current, FixedPoint target, ref FixedPoint currentVelocity,
            FixedPoint smoothTime, FixedPoint maxSpeed, FixedPoint deltaTime) {
            smoothTime = Max(FixedPoint.EN4, smoothTime);
            FixedPoint num = (FixedPoint) 2f / smoothTime;
            FixedPoint num2 = num * deltaTime;
            FixedPoint num3 = FixedPoint.One / (((FixedPoint.One + num2) + (((FixedPoint) 0.48f * num2) * num2)) +
                                                ((((FixedPoint) 0.235f * num2) * num2) * num2));
            FixedPoint num4 = current - target;
            FixedPoint num5 = target;
            FixedPoint max = maxSpeed * smoothTime;
            num4 = Clamp(num4, -max, max);
            target = current - num4;
            FixedPoint num7 = (currentVelocity + (num * num4)) * deltaTime;
            currentVelocity = (currentVelocity - (num * num7)) * num3;
            FixedPoint num8 = target + ((num4 + num7) * num3);
            if (((num5 - current) > FixedPoint.Zero) == (num8 > num5)) {
                num8 = num5;
                currentVelocity = (num8 - num5) / deltaTime;
            }

            return num8;
        }

        public static FixedPoint Dot(TSVector2 u, TSVector2 v) {
            return u.x * v.x + u.y * v.y;
        }

        public static FixedPoint Dot(TSVector lhs, TSVector rhs) {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        public static TSVector Cross(TSVector lhs, TSVector rhs) {
            return new TSVector(lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x
            );
        }

        public static FixedPoint Cross2D(TSVector2 u, TSVector2 v) {
            return u.x * v.y - u.y * v.x;
        }

        public static FixedPoint Dot2D(TSVector2 u, TSVector2 v) {
            return u.x * v.x + u.y * v.y;
        }
    }
}