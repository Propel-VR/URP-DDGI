using Unity.Mathematics;
using UnityEngine.Rendering;
using UnityEngine;
using static Unity.Mathematics.math;
using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using UnityEngine.Assertions;
using System.Runtime.CompilerServices;

namespace DDGIURP
{
    public static class BVHUtils
    {
        public const float BVH_FAR = 1e30f;
        public const int BVHBINS = 8;   // Binned BVH bin count
        public const int C_INT = 1;     // SAH heuristic
        public const int C_TRAV = 1;    // SAH heuristic

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

    public static class NativeArrayExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static T* GetPointerAt<T>(this NativeArray<T> array, int index) where T : unmanaged
        {
            Assert.IsFalse(index < 0 || index >= array.Length, "NativeArray.GetPointerAt: Index is out of range.");

            void* buffer = NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(array);
            return (T*)buffer + index;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static T* GetPointerAt<T>(this NativeArray<T> array, uint index) where T : unmanaged
        {
            Assert.IsFalse(index < 0 || index >= array.Length, "NativeArray.GetPointerAt: Index is out of range.");

            void* buffer = NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(array);
            return (T*)buffer + index;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static T* GetPointerAt<T>(this UnsafeList<T> list, int index) where T : unmanaged
        {
            Assert.IsFalse(index < 0 || index >= list.Length, "NativeArray.GetPointerAt: Index is out of range.");

            void* buffer = list.Ptr;
            return (T*)buffer + index;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static T* GetPointerAt<T>(this UnsafeList<T> list, uint index) where T : unmanaged
        {
            Assert.IsFalse(index < 0 || index >= list.Length, "NativeArray.GetPointerAt: Index is out of range.");

            void* buffer = list.Ptr;
            return (T*)buffer + index;

        }
    }

    public static class Float3Extensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float HalfArea(this float3 b) => b.x < -BVHUtils.BVH_FAR ? 0 : (b.x * b.y + b.y * b.z + b.z * b.x);
    }
}