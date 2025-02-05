using Unity.Mathematics;
using UnityEngine.Rendering;
using UnityEngine;
using static Unity.Mathematics.math;

namespace DDGIURP
{
    public static class BVHUtils
    {
        public const float BVH_FAR = 1e30f;

        public static float SA (float3 aabbMin, float3 aabbMax)
        {
            float3 e = aabbMax - aabbMin; // extent of the node
            return e.x * e.y + e.y * e.z + e.z * e.x;
        }

        public static float IntersectAABB(ref Ray ray, float3 aabbMin, float3 aabbMax)
        {
	        // "slab test" ray/AABB intersection
	        float tx1 = (aabbMin.x - ray.O.x) * ray.rD.x, tx2 = (aabbMax.x - ray.O.x) * ray.rD.x;
                float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
                float ty1 = (aabbMin.y - ray.O.y) * ray.rD.y, ty2 = (aabbMax.y - ray.O.y) * ray.rD.y;
                tmin = max(tmin, min(ty1, ty2) );
                tmax = min(tmax, max(ty1, ty2) );
                float tz1 = (aabbMin.z - ray.O.z) * ray.rD.z, tz2 = (aabbMax.z - ray.O.z) * ray.rD.z;
                tmin = max(tmin, min(tz1, tz2) );
                tmax = min(tmax, max(tz1, tz2) );

	        if (tmax >= tmin && tmin < ray.hit.t && tmax >= 0) 
                return tmin; 
            else 
                return BVH_FAR;
        }

        public static bool IntersectAABB(float3 bmin, float3 bmax, float3 aabbMin, float3 aabbMax)
        {
	        return bmin.x<aabbMax.x && bmax.x> aabbMin.x && bmin.y<aabbMax.y && bmax.y> aabbMin.y && bmin.z<aabbMax.z && bmax.z> aabbMin.z;
        }
    }
}