/*
* FarseerUnity based on Farseer Physics Engine port:
* Copyright (c) 2012 Gabriel Ochsenhofer https://github.com/gabstv/Farseer-Unity3D
*
* Original source Box2D:
* Copyright (c) 2011 Ian Qvist http://farseerphysics.codeplex.com/
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using vFrame.Lockstep.Core;

public static class FSHelper
{
	public static float ang;
	public static float len;
	public static UnityEngine.Vector3 outputV3;
	public static TSVector2 Vector3ToTSVector2(UnityEngine.Vector3 input)
	{
		return new TSVector2(input.x, input.y);
	}
	public static UnityEngine.Vector3 TSVector3ToVector3(TSVector input)
	{
		return new UnityEngine.Vector3((float)input.x, (float)input.y, (float)input.z);
	}
	public static UnityEngine.Vector3 TSVector2ToVector3(TSVector2 input)
	{
		return new UnityEngine.Vector3((float)input.x, (float)input.y, 0f);
	}
	public static TSVector2 Vector2ToTSVector2(UnityEngine.Vector2 input)
	{
		return new TSVector2(input.x, input.y);
	}
	public static UnityEngine.Vector3 LocalTranslatedVec3(UnityEngine.Vector3 input, UnityEngine.Transform localObj)
	{
		outputV3 = new UnityEngine.Vector3();
		ang = UnityEngine.Mathf.Atan2(input.y, input.x);
		ang += localObj.localRotation.eulerAngles.z * UnityEngine.Mathf.Deg2Rad;
		len = input.magnitude;
		outputV3.x = UnityEngine.Mathf.Cos(ang) * len;
		outputV3.y = UnityEngine.Mathf.Sin(ang) * len;
		//UnityEngine.Debug.Log(outputV3);
		return outputV3 + localObj.localPosition;
	}
}
