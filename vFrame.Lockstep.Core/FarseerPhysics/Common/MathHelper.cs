#if (!XNA)

#region License

/*
MIT License
Copyright ?2006 The Mono.Xna Team

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#endregion License

namespace vFrame.Lockstep.Core.FarseerPhysics.Common
{
    public static class MathHelper
    {
        //public static readonly FixedPoint E = (FixedPoint) TSMath.E;
        public static readonly FixedPoint Log10E = 0.4342945f;
        public static readonly FixedPoint Log2E = 1.442695f;
        public static readonly FixedPoint Pi = (FixedPoint) TSMath.Pi;
        public static readonly FixedPoint PiOver2 = (FixedPoint) (TSMath.Pi / 2.0);
        public static readonly FixedPoint PiOver4 = (FixedPoint) (TSMath.Pi / 4.0);
        public static readonly FixedPoint TwoPi = (FixedPoint) (TSMath.Pi * 2.0);

        public static FixedPoint Barycentric(FixedPoint value1, FixedPoint value2, FixedPoint value3, FixedPoint amount1, FixedPoint amount2) {
            return value1 + (value2 - value1) * amount1 + (value3 - value1) * amount2;
        }

        public static FixedPoint CatmullRom(FixedPoint value1, FixedPoint value2, FixedPoint value3, FixedPoint value4, FixedPoint amount) {
            // Using formula from http://www.mvps.org/directx/articles/catmull/
            // Internally using doubles not to lose precission
            FixedPoint amountSquared = amount * amount;
            FixedPoint amountCubed = amountSquared * amount;
            return (FixedPoint) (0.5 * (2.0 * value2 +
                                   (value3 - value1) * amount +
                                   (2.0 * value1 - 5.0 * value2 + 4.0 * value3 - value4) * amountSquared +
                                   (3.0 * value2 - value1 - 3.0 * value3 + value4) * amountCubed));
        }

        public static FixedPoint Clamp(FixedPoint value, FixedPoint min, FixedPoint max) {
            // First we check to see if we're greater than the max
            value = (value > max) ? max : value;

            // Then we check to see if we're less than the min.
            value = (value < min) ? min : value;

            // There's no check to see if min > max.
            return value;
        }

        public static FixedPoint Distance(FixedPoint value1, FixedPoint value2) {
            return TSMath.Abs(value1 - value2);
        }

        public static FixedPoint Hermite(FixedPoint value1, FixedPoint tangent1, FixedPoint value2, FixedPoint tangent2, FixedPoint amount) {
            // All transformed to FixedPoint not to lose precission
            // Otherwise, for high numbers of param:amount the result is NaN instead of Infinity
            FixedPoint v1 = value1, v2 = value2, t1 = tangent1, t2 = tangent2, s = amount, result;
            FixedPoint sCubed = s * s * s;
            FixedPoint sSquared = s * s;

            if (amount == 0f)
                result = value1;
            else if (amount == 1f)
                result = value2;
            else
                result = (2 * v1 - 2 * v2 + t2 + t1) * sCubed +
                         (3 * v2 - 3 * v1 - 2 * t1 - t2) * sSquared +
                         t1 * s +
                         v1;
            return (FixedPoint) result;
        }

        public static FixedPoint Lerp(FixedPoint value1, FixedPoint value2, FixedPoint amount) {
            return value1 + (value2 - value1) * amount;
        }

        public static FixedPoint Max(FixedPoint value1, FixedPoint value2) {
            return TSMath.Max(value1, value2);
        }

        public static FixedPoint Min(FixedPoint value1, FixedPoint value2) {
            return TSMath.Min(value1, value2);
        }

        public static FixedPoint SmoothStep(FixedPoint value1, FixedPoint value2, FixedPoint amount) {
            // It is expected that 0 < amount < 1
            // If amount < 0, return value1
            // If amount > 1, return value2
            FixedPoint result = Clamp(amount, 0f, 1f);
            result = Hermite(value1, 0f, value2, 0f, result);
            return result;
        }

        public static FixedPoint ToDegrees(FixedPoint radians) {
            // This method uses FixedPoint precission internally,
            // though it returns single FixedPoint
            // Factor = 180 / pi
            return (FixedPoint) (radians * 57.295779513082320876798154814105);
        }

        public static FixedPoint ToRadians(FixedPoint degrees) {
            // This method uses FixedPoint precission internally,
            // though it returns single FixedPoint
            // Factor = pi / 180
            return (FixedPoint) (degrees * 0.017453292519943295769236907684886);
        }

        /*
        public static FixedPoint WrapAngle(FixedPoint angle) {
            angle = (FixedPoint) TSMath.IEEERemainder((FixedPoint) angle, 6.2831854820251465); //2xPi precission is FixedPoint
            if (angle <= -3.141593f) {
                angle += 6.283185f;
                return angle;
            }

            if (angle > 3.141593f) {
                angle -= 6.283185f;
            }

            return angle;
        }
        */
    }
}

#endif