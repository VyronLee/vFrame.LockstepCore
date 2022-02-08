using System;

namespace vFrame.Lockstep.Core
{
    public class TSTime
    {
        private DateTime _initTime;
        private DateTime _lastFrameTime;

        /// The total number of frames that have passed (Read Only).
        public int frameCount { get; private set; }

        /// The time in seconds it took to complete the last frame (Read Only).
        public FixedPoint deltaTime { get; private set; }

        /// The time this frame has started (Read Only). This is the time in seconds since the last level has been loaded.
        public FixedPoint timeSinceLevelLoad { get; private set; }

        /// The real time in seconds since the game started (Read Only).
        public FixedPoint realtimeSinceStartup =>
            (DateTime.Now - _initTime).TotalSeconds;

        public FixedPoint realtimeSinceStartupMS =>
            (DateTime.Now - _initTime).TotalMilliseconds;

        public void Init() {
            _initTime = DateTime.Now;
        }

        public void Update() {
            var now = DateTime.Now;
            deltaTime = (now - _lastFrameTime).TotalSeconds;
            timeSinceLevelLoad = (now - _initTime).TotalSeconds;
            frameCount++;
            _lastFrameTime = now;
        }
    }
}