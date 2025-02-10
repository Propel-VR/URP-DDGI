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
    public class BLAS
    {
        public Mesh sourceMesh;
        public int index;
        public int bufferIndex;
        public int bufferCount;

        public BVH bvh;

        public BLAS(Mesh bVH, int index, int bufferIndex, int bufferCount)
        {
            sourceMesh = bVH;
            this.index = index;
            this.bufferIndex = bufferIndex;
            this.bufferCount = bufferCount;
        }
    }

    public struct Ray
    {
        public float3 O;
        public float3 D;
        public float3 rD;
        public uint instIdx;

        public Intersection hit;
    }

    public struct Intersection
    {
        public float t;
        public float2 barycentric;
        public uint inst;
        public uint prim;
        public uint steps;
    }
}
