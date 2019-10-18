/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
*/

namespace vFrame.Lockstep.Core.Physics2D
{
    /// <summary>
    /// Convert units between display and simulation units.
    /// </summary>
    public static class ConvertUnits
    {
        private static FixedPoint _displayUnitsToSimUnitsRatio = 100;
        private static FixedPoint _simUnitsToDisplayUnitsRatio = 1 / _displayUnitsToSimUnitsRatio;

        public static void SetDisplayUnitToSimUnitRatio(FixedPoint displayUnitsPerSimUnit)
        {
            _displayUnitsToSimUnitsRatio = displayUnitsPerSimUnit;
            _simUnitsToDisplayUnitsRatio = 1 / displayUnitsPerSimUnit;
        }

        public static FixedPoint ToDisplayUnits(FixedPoint simUnits)
        {
            return simUnits * _displayUnitsToSimUnitsRatio;
        }

        public static FixedPoint ToDisplayUnits(int simUnits)
        {
            return simUnits * _displayUnitsToSimUnitsRatio;
        }

        public static TSVector2 ToDisplayUnits(TSVector2 simUnits)
        {
            return simUnits * _displayUnitsToSimUnitsRatio;
        }

        public static void ToDisplayUnits(ref TSVector2 simUnits, out TSVector2 displayUnits)
        {
            TSVector2.Multiply(ref simUnits, _displayUnitsToSimUnitsRatio, out displayUnits);
        }

        public static TSVector ToDisplayUnits(TSVector simUnits)
        {
            return simUnits * _displayUnitsToSimUnitsRatio;
        }

        public static TSVector2 ToDisplayUnits(FixedPoint x, FixedPoint y)
        {
            return new TSVector2(x, y) * _displayUnitsToSimUnitsRatio;
        }

        public static void ToDisplayUnits(FixedPoint x, FixedPoint y, out TSVector2 displayUnits)
        {
            displayUnits = TSVector2.zero;
            displayUnits.x = x * _displayUnitsToSimUnitsRatio;
            displayUnits.y = y * _displayUnitsToSimUnitsRatio;
        }

        public static FixedPoint ToSimUnits(FixedPoint displayUnits)
        {
            return displayUnits * _simUnitsToDisplayUnitsRatio;
        }

        public static FixedPoint ToSimUnits(int displayUnits)
        {
            return displayUnits * _simUnitsToDisplayUnitsRatio;
        }

        public static TSVector2 ToSimUnits(TSVector2 displayUnits)
        {
            return displayUnits * _simUnitsToDisplayUnitsRatio;
        }

        public static TSVector ToSimUnits(TSVector displayUnits)
        {
            return displayUnits * _simUnitsToDisplayUnitsRatio;
        }

        public static void ToSimUnits(ref TSVector2 displayUnits, out TSVector2 simUnits)
        {
            TSVector2.Multiply(ref displayUnits, _simUnitsToDisplayUnitsRatio, out simUnits);
        }

        public static TSVector2 ToSimUnits(FixedPoint x, FixedPoint y)
        {
            return new TSVector2(x, y) * _simUnitsToDisplayUnitsRatio;
        }

        public static void ToSimUnits(FixedPoint x, FixedPoint y, out TSVector2 simUnits)
        {
            simUnits = TSVector2.zero;
            simUnits.x = x * _simUnitsToDisplayUnitsRatio;
            simUnits.y = y * _simUnitsToDisplayUnitsRatio;
        }
    }
}