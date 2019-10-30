using System;
using System.Security.Cryptography;
using System.Text;

namespace vFrame.Lockstep.Core.Physics2D
{
    public abstract class ChecksumExtractor
    {
        private static WorldChecksumExtractor worldExtractor;

        protected IPhysicsManagerBase physicsManager;

        protected abstract string GetChecksum();

        public ChecksumExtractor(IPhysicsManagerBase physicsManager) {
            this.physicsManager = physicsManager;
        }

        public static void Init(IPhysicsManagerBase physicsManager) {
            ChecksumExtractor.worldExtractor = new WorldChecksumExtractor(physicsManager);
        }

        public static string GetEncodedChecksum() {
            return GetMd5Sum(ChecksumExtractor.worldExtractor.GetChecksum());
        }

        public static string GetMd5Sum(string str) {
            Encoder encoder = Encoding.Unicode.GetEncoder();
            byte[] array = new byte[str.Length * 2];
            encoder.GetBytes(str.ToCharArray(), 0, str.Length, array, 0, true);
            MD5 mD = new MD5CryptoServiceProvider();
            byte[] array2 = mD.ComputeHash(array);
            StringBuilder stringBuilder = new StringBuilder();
            for (int i = 0; i < array2.Length; i++) {
                stringBuilder.Append(array2[i].ToString("X2"));
            }

            return stringBuilder.ToString();
        }
    }
}