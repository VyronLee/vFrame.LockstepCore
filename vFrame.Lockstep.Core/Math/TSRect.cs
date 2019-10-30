using System;

namespace vFrame.Lockstep.Core
{
    /// <summary>
    ///     <para>A 2D TSRectangle defined by X and Y position, width and height.</para>
    /// </summary>
    public struct TSRect : IEquatable<TSRect>
    {
        /// <summary>
        ///     <para>Creates a new rectangle.</para>
        /// </summary>
        /// <param name="x">The X value the rect is measured from.</param>
        /// <param name="y">The Y value the rect is measured from.</param>
        /// <param name="width">The width of the rectangle.</param>
        /// <param name="height">The height of the rectangle.</param>
        public TSRect(float x, float y, float width, float height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        /// <summary>
        ///     <para>Creates a new rectangle.</para>
        /// </summary>
        /// <param name="x">The X value the rect is measured from.</param>
        /// <param name="y">The Y value the rect is measured from.</param>
        /// <param name="width">The width of the rectangle.</param>
        /// <param name="height">The height of the rectangle.</param>
        public TSRect(FixedPoint x, FixedPoint y, FixedPoint width, FixedPoint height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        /// <summary>
        ///     <para>Creates a rectangle given a size and position.</para>
        /// </summary>
        /// <param name="position">The position of the minimum corner of the rect.</param>
        /// <param name="size">The width and height of the rect.</param>
        public TSRect(TSVector2 position, TSVector2 size) {
            x = position.x;
            y = position.y;
            width = size.x;
            height = size.y;
        }

        /// <summary>
        ///     <para></para>
        /// </summary>
        /// <param name="source"></param>
        public TSRect(TSRect source) {
            x = source.x;
            y = source.y;
            width = source.width;
            height = source.height;
        }

        /// <summary>
        ///     <para>Shorthand for writing new TSRect(0,0,0,0).</para>
        /// </summary>
        public static TSRect zero => new TSRect(0.0f, 0.0f, 0.0f, 0.0f);

        /// <summary>
        ///     <para>Creates a rectangle from min/max coordinate values.</para>
        /// </summary>
        /// <param name="xmin">The minimum X coordinate.</param>
        /// <param name="ymin">The minimum Y coordinate.</param>
        /// <param name="xmax">The maximum X coordinate.</param>
        /// <param name="ymax">The maximum Y coordinate.</param>
        /// <returns>
        ///     <para>A rectangle matching the specified coordinates.</para>
        /// </returns>
        public static TSRect MinMaxRect(FixedPoint xmin, FixedPoint ymin, FixedPoint xmax,
            FixedPoint ymax) {
            return new TSRect(xmin, ymin, xmax - xmin, ymax - ymin);
        }

        /// <summary>
        ///     <para>Set components of an existing TSRect.</para>
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        public void Set(FixedPoint x, FixedPoint y, FixedPoint width, FixedPoint height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        /// <summary>
        ///     <para>The X coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint x { get; set; }

        /// <summary>
        ///     <para>The Y coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint y { get; set; }

        /// <summary>
        ///     <para>The X and Y position of the rectangle.</para>
        /// </summary>
        public TSVector2 position {
            get { return new TSVector2(x, y); }
            set {
                x = value.x;
                y = value.y;
            }
        }

        /// <summary>
        ///     <para>The position of the center of the rectangle.</para>
        /// </summary>
        public TSVector2 center {
            get { return new TSVector2(x + width / 2f, y + height / 2f); }
            set {
                x = value.x - width / 2f;
                y = value.y - height / 2f;
            }
        }

        /// <summary>
        ///     <para>The position of the minimum corner of the rectangle.</para>
        /// </summary>
        public TSVector2 min {
            get { return new TSVector2(xMin, yMin); }
            set {
                xMin = value.x;
                yMin = value.y;
            }
        }

        /// <summary>
        ///     <para>The position of the maximum corner of the rectangle.</para>
        /// </summary>
        public TSVector2 max {
            get { return new TSVector2(xMax, yMax); }
            set {
                xMax = value.x;
                yMax = value.y;
            }
        }

        /// <summary>
        ///     <para>The width of the rectangle, measured from the X position.</para>
        /// </summary>
        public FixedPoint width { get; set; }

        /// <summary>
        ///     <para>The height of the rectangle, measured from the Y position.</para>
        /// </summary>
        public FixedPoint height { get; set; }

        /// <summary>
        ///     <para>The width and height of the rectangle.</para>
        /// </summary>
        public TSVector2 size {
            get { return new TSVector2(width, height); }
            set {
                width = value.x;
                height = value.y;
            }
        }

        /// <summary>
        ///     <para>The minimum X coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint xMin {
            get { return x; }
            set {
                var xMax = this.xMax;
                x = value;
                width = xMax - x;
            }
        }

        /// <summary>
        ///     <para>The minimum Y coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint yMin {
            get { return y; }
            set {
                var yMax = this.yMax;
                y = value;
                height = yMax - y;
            }
        }

        /// <summary>
        ///     <para>The maximum X coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint xMax {
            get { return width + x; }
            set { width = value - x; }
        }

        /// <summary>
        ///     <para>The maximum Y coordinate of the rectangle.</para>
        /// </summary>
        public FixedPoint yMax {
            get { return height + y; }
            set { height = value - y; }
        }

        /// <summary>
        ///     <para>
        ///         Returns true if the x and y components of point is a point inside
        ///         this rectangle. If allowInverse is present and true, the width and
        ///         height of the TSRect are allowed to take negative values (ie, the min
        ///         value is greater than the max), and the test will still work.
        ///     </para>
        /// </summary>
        /// <param name="point">Point to test.</param>
        /// <param name="allowInverse">
        ///     Does the test allow the TSRect's width and height to
        ///     be negative?
        /// </param>
        /// <returns>
        ///     <para>True if the point lies within the specified rectangle.</para>
        /// </returns>
        public bool Contains(TSVector2 point) {
            return (double) point.x >= xMin && (double) point.x < xMax &&
                   (double) point.y >= yMin && (double) point.y < yMax;
        }

        /// <summary>
        ///     <para>
        ///         Returns true if the x and y components of point is a point inside
        ///         this rectangle. If allowInverse is present and true, the width and
        ///         height of the TSRect are allowed to take negative values (ie, the min
        ///         value is greater than the max), and the test will still work.
        ///     </para>
        /// </summary>
        /// <param name="point">Point to test.</param>
        /// <param name="allowInverse">
        ///     Does the test allow the TSRect's width and height to
        ///     be negative?
        /// </param>
        /// <returns>
        ///     <para>True if the point lies within the specified rectangle.</para>
        /// </returns>
        public bool Contains(TSVector point) {
            return (double) point.x >= xMin && (double) point.x < xMax &&
                   (double) point.y >= yMin && (double) point.y < yMax;
        }

        /// <summary>
        ///     <para>
        ///         Returns true if the x and y components of point is a point inside
        ///         this rectangle. If allowInverse is present and true, the width and
        ///         height of the TSRect are allowed to take negative values (ie, the min
        ///         value is greater than the max), and the test will still work.
        ///     </para>
        /// </summary>
        /// <param name="point">Point to test.</param>
        /// <param name="allowInverse">
        ///     Does the test allow the TSRect's width and height to
        ///     be negative?
        /// </param>
        /// <returns>
        ///     <para>True if the point lies within the specified rectangle.</para>
        /// </returns>
        public bool Contains(TSVector point, bool allowInverse) {
            if (!allowInverse)
                return Contains(point);
            var flag = false;
            if (width < 0.0 && (double) point.x <= xMin &&
                (double) point.x > xMax || width >= 0.0 &&
                (double) point.x >= xMin && (double) point.x < xMax)
                flag = true;
            return flag &&
                   (height < 0.0 && (double) point.y <= yMin &&
                    (double) point.y > yMax || height >= 0.0 &&
                    (double) point.y >= yMin && (double) point.y < yMax);
        }

        private static TSRect OrderMinMax(TSRect rect) {
            if (rect.xMin > (double) rect.xMax) {
                var xMin = rect.xMin;
                rect.xMin = rect.xMax;
                rect.xMax = xMin;
            }

            if (rect.yMin > (double) rect.yMax) {
                var yMin = rect.yMin;
                rect.yMin = rect.yMax;
                rect.yMax = yMin;
            }

            return rect;
        }

        /// <summary>
        ///     <para>
        ///         Returns true if the other rectangle overlaps this one. If
        ///         allowInverse is present and true, the widths and heights of the TSRects
        ///         are allowed to take negative values (ie, the min value is greater than
        ///         the max), and the test will still work.
        ///     </para>
        /// </summary>
        /// <param name="other">Other rectangle to test overlapping with.</param>
        /// <param name="allowInverse">
        ///     Does the test allow the widths and heights of the
        ///     TSRects to be negative?
        /// </param>
        public bool Overlaps(TSRect other) {
            return other.xMax > (double) xMin && other.xMin < (double) xMax &&
                   other.yMax > (double) yMin && other.yMin < (double) yMax;
        }

        /// <summary>
        ///     <para>
        ///         Returns true if the other rectangle overlaps this one. If
        ///         allowInverse is present and true, the widths and heights of the TSRects
        ///         are allowed to take negative values (ie, the min value is greater than
        ///         the max), and the test will still work.
        ///     </para>
        /// </summary>
        /// <param name="other">Other rectangle to test overlapping with.</param>
        /// <param name="allowInverse">
        ///     Does the test allow the widths and heights of the
        ///     TSRects to be negative?
        /// </param>
        public bool Overlaps(TSRect other, bool allowInverse) {
            var rect = this;
            if (allowInverse) {
                rect = OrderMinMax(rect);
                other = OrderMinMax(other);
            }

            return rect.Overlaps(other);
        }

        /// <summary>
        ///     <para>Returns a point inside a rectangle, given normalized coordinates.</para>
        /// </summary>
        /// <param name="rectangle">TSRectangle to get a point inside.</param>
        /// <param name="normalizedTSRectCoordinates">
        ///     Normalized coordinates to get a point
        ///     for.
        /// </param>
        public static TSVector2 NormalizedToPoint(
            TSRect rectangle,
            TSVector2 normalizedTSRectCoordinates) {
            return new TSVector2(
                TSMath.Lerp(rectangle.x, rectangle.xMax,
                    normalizedTSRectCoordinates.x),
                TSMath.Lerp(rectangle.y, rectangle.yMax,
                    normalizedTSRectCoordinates.y));
        }

        /// <summary>
        ///     <para>Returns the normalized coordinates cooresponding the the point.</para>
        /// </summary>
        /// <param name="rectangle">TSRectangle to get normalized coordinates inside.</param>
        /// <param name="point">
        ///     A point inside the rectangle to get normalized coordinates
        ///     for.
        /// </param>
        public static TSVector2 PointToNormalized(TSRect rectangle,
            TSVector2 point) {
            return new TSVector2(
                TSMath.InverseLerp(rectangle.x, rectangle.xMax, point.x),
                TSMath.InverseLerp(rectangle.y, rectangle.yMax, point.y));
        }

        public static bool operator !=(TSRect lhs, TSRect rhs) {
            return !(lhs == rhs);
        }

        public static bool operator ==(TSRect lhs, TSRect rhs) {
            return lhs.x == (double) rhs.x && lhs.y == (double) rhs.y &&
                   lhs.width == (double) rhs.width &&
                   lhs.height == (double) rhs.height;
        }

        public override int GetHashCode() {
            return x.GetHashCode() ^ (width.GetHashCode() << 2) ^
                   (y.GetHashCode() >> 2) ^ (height.GetHashCode() >> 1);
        }

        public override bool Equals(object other) {
            if (!(other is TSRect))
                return false;
            return Equals((TSRect) other);
        }

        public bool Equals(TSRect other) {
            return x.Equals(other.x) && y.Equals(other.y) &&
                   width.Equals(other.width) && height.Equals(other.height);
        }

        /// <summary>
        ///     <para>Returns a nicely formatted string for this TSRect.</para>
        /// </summary>
        /// <param name="format"></param>
        public override string ToString() {
            return string.Format(
                "(x:{0:F2}, y:{1:F2}, width:{2:F2}, height:{3:F2})", (object) x,
                (object) y, (object) width, (object) height);
        }

        /// <summary>
        ///     <para>Returns a nicely formatted string for this TSRect.</para>
        /// </summary>
        /// <param name="format"></param>
        public string ToString(string format) {
            return string.Format("(x:{0}, y:{1}, width:{2}, height:{3})",
                (object) x.ToString(format), (object) y.ToString(format),
                (object) width.ToString(format),
                (object) height.ToString(format));
        }
    }
}