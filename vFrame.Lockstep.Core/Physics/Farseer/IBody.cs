using System;

namespace vFrame.Lockstep.Core
{
    public interface IBody
    {
        bool TSDisabled { get; set; }

        string Checkum();
    }
}