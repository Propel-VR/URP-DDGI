using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static DDGIURP.BVHUtils;
using static Unity.Mathematics.math;

namespace DDGIURP
{
    /// <summary>
    /// BLAS instance. 
    /// These are the primitives (instead of triangles) that will be used when building a TLAS.
    /// </summary>
    public struct BLASInstance
    {
        public float4x4 localToWorld;
        public float4x4 worldToLocal;
        public float3 aabbMin;
        public float3 aabbMax;
        public uint blasIdx;
    }

    /// <summary>
    /// 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
    /// When aligned to a cache line boundary, two of these fit together.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct BVHNode
    {
        [FieldOffset(0)] public float3 aabbMin;
        [FieldOffset(12)] public uint leftFirst; // 16 bytes
        [FieldOffset(16)] public float3 aabbMax;
        [FieldOffset(28)] public uint triCount; // 16 bytes, total: 32 bytes
        public readonly bool IsLeaf => triCount > 0;
        public readonly float SurfaceArea => SA(aabbMin, aabbMax);
        public readonly float Intersect(Ray ray) => IntersectAABB(ref ray, aabbMin, aabbMax);
        public readonly bool Intersect(float3 bmin, float3 bmax) => IntersectAABB(bmin, bmax, aabbMin, aabbMax);
    }

    /// <summary>
    /// A fragment stores the bounds of an input primitive. The name 'Fragment' is from
    /// "Parallel Spatial Splits in Bounding Volume Hierarchies", 2016, Fuetterling et al.,
    /// and refers to the potential splitting of these boxes for SBVH construction.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct Fragment
    {
        [FieldOffset(0)] public float3 bmin; // AABB min x, y, z
        [FieldOffset(12)] public uint primIdx; // Index of the original primitive
        [FieldOffset(16)] public float3 bmax; // AABB max x, y, z
        [FieldOffset(28)] public uint clipped; // Fragment is the result of clipping if > 0
        public readonly bool ValidBox => bmin.x < BVH_FAR;
    }

    /// <summary>
    /// Based on tinyBVH's BVH class.
    /// 
    /// Builds an unoptimized BVH. Burst-compatible. 
    /// To be converted to CWBVH8 for GPU raytracing.
    /// 
    /// These BVH can be use to build BLAS or TLAS
    /// 
    /// To build only a slice of a large mesh, use the NativeArray<>.Slice method.
    /// 
    /// .NET doesn't play well with uint, so there might be some casting.
    /// Really hope Unity adds uint support with NativeArrays.
    /// </summary>
    [BurstCompile]
    unsafe public struct BVH : IDisposable
    {
        [NativeDisableUnsafePtrRestriction] public float3* vertices;             // Primitive array
        [NativeDisableUnsafePtrRestriction] public uint* indices;                // Vertex indices (only uses if BVH built over indexed prims)
        [NativeDisableUnsafePtrRestriction] public BLASInstance* blasInstances;  // BLAS array
        public UnsafeList<uint> primitives;             // Primitive index array
        public UnsafeList<BVHNode> bvhNode;             // BVH node pool. Root is always node 0.
        public UnsafeList<Fragment> fragments;          // Input primitive bounding boxes

        bool isTLAS;
        bool isIndexed;
        uint primCount;
        uint nodeCount;
        float3 aabbMin, aabbMax;

        public readonly bool IsTLAS => isTLAS;
        public readonly bool IsIndexed => isIndexed;
        public readonly uint AllocatedNodes => (uint)bvhNode.Length;
        public readonly uint TrisCount => primCount;
        public readonly uint InstCount => primCount;
        public readonly uint PrimCount => primCount;
        public readonly uint NodeCount => nodeCount;
        public readonly bool IsCreated => bvhNode.IsCreated;

        public void Dispose() {
            if (primitives.IsCreated) primitives.Dispose();
            if (bvhNode.IsCreated) bvhNode.Dispose();
            if (fragments.IsCreated) fragments.Dispose();
        }

        public void Build (NativeArray<float3> vertices, Allocator allocator)
        {
            Assert.IsTrue(vertices.Length != 0, "BVH.Allocate: Empty mesh with no verticies.");
            isTLAS = false;
            isIndexed = false;
            this.vertices = (float3*)vertices.GetUnsafeReadOnlyPtr();
            this.indices = null;
            this.blasInstances = null;

            int trisCount = vertices.Length / 3;
            int spaceNeeded = trisCount * 2;
            primCount = (uint)trisCount;

            // Allocate memory on first build
            bvhNode = new UnsafeList<BVHNode>(spaceNeeded, allocator, NativeArrayOptions.UninitializedMemory);
            primitives = new UnsafeList<uint>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            fragments = new UnsafeList<Fragment>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            bvhNode.Length = bvhNode.Capacity;
            primitives.Length = primitives.Capacity;
            fragments.Length = fragments.Capacity;

            PrepareBuildBLAS();
            BuildInternal();
        }

        public void Build(NativeArray<float3> vertices, NativeArray<uint> indices, Allocator allocator)
        {
            Assert.IsTrue(vertices.Length != 0, "BVH.Allocate: Empty mesh with no verticies.");
            isTLAS = false;
            isIndexed = true;
            this.vertices = (float3*)vertices.GetUnsafeReadOnlyPtr();
            this.indices = (uint*)indices.GetUnsafeReadOnlyPtr();
            this.blasInstances = null;

            int trisCount = indices.Length / 3;
            int spaceNeeded = trisCount * 2;
            primCount = (uint)trisCount;

            // Allocate memory on first build
            bvhNode = new UnsafeList<BVHNode>(spaceNeeded, allocator, NativeArrayOptions.UninitializedMemory);
            primitives = new UnsafeList<uint>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            fragments = new UnsafeList<Fragment>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            bvhNode.Length = bvhNode.Capacity;
            primitives.Length = primitives.Capacity;
            fragments.Length = fragments.Capacity;
            
            PrepareBuildBLAS();
            BuildInternal();
        }

        public void Build(NativeArray<BLASInstance> blasInstances, Allocator allocator)
        {
            Assert.IsTrue(blasInstances.Length == 0, "BVH.Allocate: Empty TLAS with no instances.");
            isTLAS = true;
            isIndexed = false;
            this.vertices = null;
            this.indices = null;
            this.blasInstances = (BLASInstance*)blasInstances.GetUnsafeReadOnlyPtr();

            int instCount = blasInstances.Length;
            int spaceNeeded = instCount * 2;
            primCount = (uint)instCount;

            // Allocate memory on first build
            bvhNode = new UnsafeList<BVHNode>(spaceNeeded, allocator, NativeArrayOptions.UninitializedMemory);
            primitives = new UnsafeList<uint>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            fragments = new UnsafeList<Fragment>((int)primCount, allocator, NativeArrayOptions.UninitializedMemory);
            bvhNode.Length = bvhNode.Capacity;
            primitives.Length = primitives.Capacity;
            fragments.Length = fragments.Capacity;

            PrepareBuildTLAS();
            BuildInternal();
        }

        void PrepareBuildBLAS ()
        {
            uint trisCount = primCount;


            BVHNode* root = bvhNode.GetPointerAt(0);
            root->leftFirst = 0;
            root->triCount = TrisCount;
            root->aabbMin = float3(BVH_FAR);
            root->aabbMax = float3(-BVH_FAR);

            Assert.IsTrue(trisCount != 0, "BVH.Build: Empty mesh with no verticies or indicies");
            if(IsIndexed)
            {
                // Building an AABB over triangles consisting of vertices indexed by 'indices'.
                for (int i = 0; i < trisCount; i++)
                {
                    uint i0 = indices[i * 3], 
                        i1 = indices[i * 3 + 1], 
                        i2 = indices[i * 3 + 2];
                    float3 v0 = vertices[(int)i0], 
                        v1 = vertices[(int)i1], 
                        v2 = vertices[(int)i2];

                    fragments[i] = new()
                    {
                        bmin = min(v0, min(v1, v2)),
                        bmax = max(v0, max(v1, v2)),
                    };

                    root->aabbMin = min(root->aabbMin, fragments[i].bmin);
                    root->aabbMax = max(root->aabbMax, fragments[i].bmax); 
                    primitives[i] = (uint)i;
                }
            }
            else
            {
                // Building an AABB over triangles specified as three 12-byte vertices each.
                for (int i = 0; i < trisCount; i++)
                {
                    float3 v0 = vertices[i * 3], 
                        v1 = vertices[i * 3 + 1], 
                        v2 = vertices[i * 3 + 2];

                    fragments[i] = new()
                    {
                        bmin = min(v0, min(v1, v2)),
                        bmax = max(v0, max(v1, v2)),
                    }; ;

                    root->aabbMin = min(root->aabbMin, fragments[i].bmin);
                    root->aabbMax = max(root->aabbMax, fragments[i].bmax);
                    primitives[i] = (uint)i;
                }
            }
        }
    
        void PrepareBuildTLAS ()
        {
            uint instCount = primCount;


            BVHNode* root = bvhNode.GetPointerAt(0);
            root->leftFirst = 0;
            root->triCount = TrisCount;
            root->aabbMin = float3(BVH_FAR);
            root->aabbMax = float3(-BVH_FAR);

            // Building an AABB over blas instances.
            for (int i = 0; i < instCount; i++)
            {
                var blasInstance = blasInstances[i];
                fragments[i] = new()
                {
                    bmin = blasInstance.aabbMin,
                    bmax = blasInstance.aabbMax,
                    primIdx = (uint)i,
                    clipped = 0
                };

                root->aabbMin = min(root->aabbMin, fragments[i].bmin);
                root->aabbMax = max(root->aabbMax, fragments[i].bmax);
                primitives[i] = (uint)i;
            }
        }
    
        void BuildInternal ()
        {
            // Subdivide root node recursively
            var task = stackalloc uint[256];
            uint taskCount = 0;
            uint nodeIdx = 0;
            BVHNode* root = bvhNode.GetPointerAt(0);
            nodeCount = 1;

            float3 minDim = (root->aabbMax - root->aabbMin) * 1e-20f,
                bestLMin = 0, bestLMax = 0,
                bestRMin = 0, bestRMax = 0;

            // Avoiding stackalloc loop overflow
            var binMin = stackalloc float3[3 * BVHBINS];
            var binMax = stackalloc float3[3 * BVHBINS];
            var count = stackalloc uint[3 * BVHBINS];
            var lBMin = stackalloc float3[BVHBINS - 1];
            var rBMin = stackalloc float3[BVHBINS - 1];
            var lBMax = stackalloc float3[BVHBINS - 1];
            var rBMax = stackalloc float3[BVHBINS - 1];
            var ANL = stackalloc float[BVHBINS - 1];
            var ANR = stackalloc float[BVHBINS - 1];

            while (true)
            {
                while (true)
                {
                    BVHNode* node = bvhNode.GetPointerAt(nodeIdx);
                    float3 rpd3 = float3(BVHBINS / (node->aabbMax - node->aabbMin));
                    float3 nmin3 = node->aabbMin;

                    // var binMin = stackalloc float3[3 * BVHBINS];
                    // var binMax = stackalloc float3[3 * BVHBINS];
                    // var count = stackalloc uint[3 * BVHBINS];
                    for (int i = 0; i < 3 * BVHBINS; i++)
                    {
                        binMin[i] = BVH_FAR;
                        binMax[i] = -BVH_FAR;
                        count[i] = 0;
                    }

                    // Process all tris for x, y and z at once
                    for (int i = 0; i < node->triCount; i++)
                    {
                        int fi = (int)primitives[(int)node->leftFirst + i];
                        int3 bi = int3(((fragments[fi].bmin + fragments[fi].bmax) * 0.5f - nmin3) * rpd3);
                        bi.x = clamp(bi.x, 0, BVHBINS - 1);
                        bi.y = clamp(bi.y, 0, BVHBINS - 1);
                        bi.z = clamp(bi.z, 0, BVHBINS - 1);
                        binMin[BinIndex(0, bi.x)] = min(binMin[BinIndex(0, bi.x)], fragments[fi].bmin);
                        binMax[BinIndex(0, bi.x)] = max(binMax[BinIndex(0, bi.x)], fragments[fi].bmax);
                        count[BinIndex(0, bi.x)]++;
                        binMin[BinIndex(1, bi.y)] = min(binMin[BinIndex(1, bi.y)], fragments[fi].bmin);
                        binMax[BinIndex(1, bi.y)] = max(binMax[BinIndex(1, bi.y)], fragments[fi].bmax);
                        count[BinIndex(1, bi.y)]++;
                        binMin[BinIndex(2, bi.z)] = min(binMin[BinIndex(2, bi.z)], fragments[fi].bmin);
                        binMax[BinIndex(2, bi.z)] = max(binMax[BinIndex(2, bi.z)], fragments[fi].bmax);
                        count[BinIndex(2, bi.z)]++;
                    }

                    // Calculate per-split totals
                    float splitCost = BVH_FAR;
                    float rSAV = 1.0f / node->SurfaceArea;
                    uint bestAxis = 0, bestPos = 0;
                    for (uint a = 0; a < 3; a++)
                    {
                        if ((node->aabbMax[(int)a] - node->aabbMin[(int)a]) <= minDim[(int)a]) continue;

                        // var lBMin = stackalloc float3[BVHBINS - 1];
                        // var rBMin = stackalloc float3[BVHBINS - 1];
                        float3 l1 = BVH_FAR;
                        float3 l2 = -BVH_FAR;
                        // var lBMax = stackalloc float3[BVHBINS - 1];
                        // var rBMax = stackalloc float3[BVHBINS - 1];
                        float3 r1 = BVH_FAR;
                        float3 r2 = -BVH_FAR;

                        //var ANL = stackalloc float[BVHBINS - 1];
                        //var ANR = stackalloc float[BVHBINS - 1];
                        for (uint lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
                        {
                            lBMin[i] = l1 = min(l1, binMin[BinIndex(a, i)]);
                            rBMin[BVHBINS - 2 - i] = r1 = min(r1, binMin[BinIndex(a, BVHBINS - 1 - i)]);
                            lBMax[i] = l2 = max(l2, binMax[BinIndex(a, i)]);
                            rBMax[BVHBINS - 2 - i] = r2 = max(r2, binMax[BinIndex(a, BVHBINS - 1 - i)]);
                            lN += count[BinIndex(a, i)];
                            rN += count[BinIndex(a, BVHBINS - 1 - i)];
                            ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).HalfArea() * lN);
                            ANR[BVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).HalfArea() * rN);
                        }
                        // evaluate bin totals to find best position for object split
                        for (uint i = 0; i < BVHBINS - 1; i++)
                        {
                            float C = C_TRAV + rSAV * C_INT * (ANL[i] + ANR[i]);
                            if (C < splitCost)
                            {
                                splitCost = C;
                                bestAxis = a;
                                bestPos = i;
                                bestLMin = lBMin[i];
                                bestRMin = rBMin[i];
                                bestLMax = lBMax[i];
                                bestRMax = rBMax[i];
                            }
                        }
                    }
                    float noSplitCost = (float)node->triCount * C_INT;
                    if (splitCost >= noSplitCost) break; // not splitting is better.
                                                            // in-place partition
                    uint j = node->leftFirst + node->triCount;
                    uint src = node->leftFirst;
                    float rpd = rpd3[(int)bestAxis];
                    float nmin = nmin3[(int)bestAxis];
                    for (int i = 0; i < node->triCount; i++)
                    {
                        uint fi = primitives[(int)src];
                        int bi = (int)(((fragments[(int)fi].bmin[(int)bestAxis] + fragments[(int)fi].bmax[(int)bestAxis]) * 0.5f - nmin) * rpd);
                        bi = clamp(bi, 0, BVHBINS - 1);
                        if ((uint)bi <= bestPos)
                        {
                            src++;
                        }
                        else
                        {
                            j--;
                            (primitives[(int)j], primitives[(int)src]) = (primitives[(int)src], primitives[(int)j]);
                        }
                    }

                    // Create child nodes
                    uint leftCount = src - node->leftFirst, rightCount = node->triCount - leftCount;
                    if (leftCount == 0 || rightCount == 0) break; // Should not happen.
                    uint lci = nodeCount++;
                    uint rci = nodeCount++;
                    bvhNode[(int)lci] = new BVHNode()
                    {
                        aabbMin = bestLMin,
                        aabbMax = bestLMax,
                        leftFirst = node->leftFirst,
                        triCount = leftCount
                    };
                    bvhNode[(int)rci] = new BVHNode()
                    {
                        aabbMin = bestRMin,
                        aabbMax = bestRMax,
                        leftFirst = j,
                        triCount = rightCount,
                    };
                    node->leftFirst = lci;
                    node->triCount = 0;

                    // Recurse
                    task[taskCount++] = rci;
                    nodeIdx = lci;
                }
                // Fetch subdivision task from stack
                if (taskCount == 0) break; 
                else nodeIdx = task[--taskCount];
            }

            // All done.
            aabbMin = bvhNode[0].aabbMin; 
            aabbMax = bvhNode[0].aabbMax;
        }

        public uint Intersect(ref Ray ray)
        {
            BVHNode* node = bvhNode.GetPointerAt(0); 
            var stack = stackalloc BVHNode*[64];
            uint stackPtr = 0; 
            uint cost = 0;

	        while (true)
	        {
		        cost += C_TRAV;
		        if (node->IsLeaf)
		        {
                    if (IsIndexed) {
                        for (uint i = 0; i < node->triCount; i++, cost += C_INT)
                        {
                            IntersectTriIndexed(ref ray, primitives[(int)(node->leftFirst + i)]);
                        }
                    }
                    else { 
                        for (uint i = 0; i < node->triCount; i++, cost += C_INT) 
                        { 
                            IntersectTri(ref ray, primitives[(int)(node->leftFirst + i)]);
                        }
                    }
                    if (stackPtr == 0)
                    {
                        break;
                    }
                    else
                    {
                        node = stack[--stackPtr];
                    }
			        continue;
		        }
		        BVHNode* child1 = bvhNode.GetPointerAt((int)node->leftFirst);
		        BVHNode* child2 = bvhNode.GetPointerAt((int)node->leftFirst + 1);
		        float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		        if (dist1 > dist2) 
                { 
                    (dist1, dist2) = (dist2, dist1);
                    BVHNode* temp = child1;
                    child1 = child2;
                    child2 = temp;
                }
		        if (dist1 == BVH_FAR) // Missed both child nodes
                {
			        if (stackPtr == 0) break; else node = stack[--stackPtr];
		        }
                else // Hit at least one node, continue with the nearest
                {
			        node = child1;
			        if (dist2 != BVH_FAR) stack[stackPtr++] = child2; // Push far child
                }
	        }
	        return cost;
        }

        void IntersectTriIndexed(ref Ray ray, uint idx)
        {
            // Moeller-Trumbore ray/triangle intersection algorithm
            uint i0 = indices[(int)idx * 3];
            uint i1 = indices[(int)idx * 3 + 1];
            uint i2 = indices[(int)idx * 3 + 2];
	        
            float3 vert0 = vertices[(int)i0];
            float3 edge1 = vertices[(int)i1] - vert0;
            float3 edge2 = vertices[(int)i2] - vert0;
            float3 h = cross(ray.D, edge2);
	        float a = dot(edge1, h);
	        if (abs(a) < 0.0000001f) return; // Ray parallel to triangle
	        float f = 1 / a;
	        float3 s = ray.O - float3(vert0);
	        float u = f * dot(s, h);
	        if (u < 0 || u > 1) return;
            float3 q = cross(s, edge1);
	        float v = f * dot(ray.D, q);
	        if (v < 0 || u + v > 1) return;
	        float t = f * dot(edge2, q);
	        if (t > 0 && t < ray.hit.t)
	        {
                // register a hit: ray is shortened to t
                ray.hit.t = t;
                ray.hit.barycentric = float2(u, v);
                ray.hit.prim = idx;
                ray.hit.inst = ray.instIdx;
            }
        }

        void IntersectTri(ref Ray ray, uint idx)
        {
            // Moeller-Trumbore ray/triangle intersection algorithm
            uint vertIdx = idx * 3;
            float3 vert0 = vertices[(int)vertIdx];
            float3 edge1 = vertices[(int)vertIdx + 1] - vert0;
            float3 edge2 = vertices[(int)vertIdx + 2] - vert0;
            float3 h = cross(ray.D, edge2);
            float a = dot(edge1, h);

            // Ray parallel to triangle
            if (abs(a) < 0.0000001f) return;

	        float f = 1 / a;
            float3 s = ray.O - float3(vert0);
            float u = f * dot(s, h);
            if (u< 0 || u> 1) return;
            float3 q = cross(s, edge1);
            float v = f * dot(ray.D, q);
	        if (v < 0 || u + v > 1) return;

	        float t = f * dot(edge2, q);
	        if (t > 0 && t < ray.hit.t)
	        {
                // Register a hit: ray is shortened to t
                ray.hit.t = t; 
                ray.hit.barycentric = float2(u, v);
                ray.hit.prim = idx;
                ray.hit.inst = ray.instIdx;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int BinIndex(int bin, int i) => i + bin * BVHBINS;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private uint BinIndex(uint bin, uint i) => i + bin * BVHBINS;
    }
}