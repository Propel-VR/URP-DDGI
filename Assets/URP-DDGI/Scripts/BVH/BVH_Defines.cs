using System;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using static DDGIURP.BVHUtils;

namespace DDGIURP
{
    [Serializable]
    public class Blas
    {
        public Mesh BVH;
        public int blasIndex;
        public int bufferIndex;
        public int bufferCount;

        public Blas(Mesh bVH, int blasIndex, int bufferIndex, int bufferCount)
        {
            BVH = bVH;
            this.bufferIndex = bufferIndex;
            this.bufferCount = bufferCount;
        }
    }

    public struct BlasInstance
    {
        public float4x4 localToWorld;
        public float4x4 worldToLocal;
        public float3 aabbMin;
        public float3 aabbMax;
        uint blasIdx;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct BVHNode
    {
        // 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
        // When aligned to a cache line boundary, two of these fit together.
        [FieldOffset(0)] public float3 aabbMin;
        [FieldOffset(12)] public uint leftFirst; // 16 bytes
        [FieldOffset(16)] public float3 aabbMax;
        [FieldOffset(28)] public uint triCount; // 16 bytes, total: 32 bytes
        public bool IsLeaf => triCount > 0;
        public float SurfaceArea => SA(aabbMin, aabbMax);
        public float Intersect (Ray ray) => IntersectAABB(ref ray, aabbMin, aabbMax);
        public bool Intersect(float3 bmin, float3 bmax) => IntersectAABB(bmin, bmax, aabbMin, aabbMax);
    }

    public struct Ray
    {
        public float3 O;
        public float3 D;
        public float3 rD;

        public Intersection hit;
    }

    public struct Intersection
    {
        public float t;
        public float2 barycentric;
        public uint triIndex;
        public uint steps;
    }

    [BurstCompile]
    public struct BVHJob : IJob
    {
        [ReadOnly] public NativeArray<float3> vertexBuffer;
        public int bufferIndex;
        public int bufferCount;


        public void Execute ()
        {

        }
    }
}
