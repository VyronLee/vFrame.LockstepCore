using System;

namespace vFrame.Lockstep.Core
{
    public static class TSTime
    {
        private static DateTime _initTime;
        private static DateTime lastFrameTime;

        /// The total number of frames that have passed (Read Only).
        public static int frameCount { get; private set; }

        /// The time in seconds it took to complete the last frame (Read Only).
        public static FixedPoint deltaTime { get; private set; }

        /// The time this frame has started (Read Only). This is the time in seconds since the last level has been loaded.
        public static FixedPoint timeSinceLevelLoad { get; private set; }

        /// The real time in seconds since the game started (Read Only).
        public static FixedPoint realtimeSinceStartup =>
            (DateTime.Now - _initTime).TotalSeconds;

        public static FixedPoint realtimeSinceStartupMS =>
            (DateTime.Now - _initTime).TotalMilliseconds;

        public static void Init()
        {
            _initTime = DateTime.Now;
        }

        public static void Update()
        {
            var now = DateTime.Now;
            deltaTime = (now - lastFrameTime).TotalSeconds;
            timeSinceLevelLoad = (now - _initTime).TotalSeconds;
            frameCount++;
            lastFrameTime = now;
        }
    }
}