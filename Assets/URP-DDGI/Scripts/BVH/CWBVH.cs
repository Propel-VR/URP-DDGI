using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using static DDGIURP.BVHUtils;
using static Unity.Mathematics.math;

namespace DDGIURP
{
    public unsafe struct MBVHNode {
        public float3 aabbMin;
        public float3 aabbMax;
        public uint firstTri;
        public uint triCount;
        public fixed uint child[8];
        public uint childCount;

        public readonly bool IsLeaf => triCount > 0;
    }

    /// <summary>
    /// Converts BVH to the format specified in "Efficient Incoherent Ray
    /// Traversal on GPUs Through Comopressed Wide BVHs", Wlitie et al. 2017.
    /// Adapted from tinyBVH.h, which is itself adapted from code by "AlanWBFT"
    /// 
    /// Unlike tinyBVH.h, this struct only allows for conversion from other formats.
    /// </summary>
    [BurstCompile]
    public unsafe struct CWBVH : IDisposable
    {
        public BVH bvh;

        public UnsafeList<float4> bvh8Data;      // Nodes in CWBVH8 format
        public UnsafeList<float4> bvh8Tris;     // Triangles for the CWBVH8 nodes

        public void Dispose ()
        {
            if (bvh8Data.IsCreated) bvh8Data.Dispose();
            if (bvh8Tris.IsCreated) bvh8Tris.Dispose();
        }

        public void Build (BVH bvh, Allocator allocator)
        {
            this.bvh = bvh;

            BuildInternalMBVH(out UnsafeList<MBVHNode> mbvhNode, out uint mbvhNodeCount);
            BuildInternalCWBVH(ref mbvhNode, ref mbvhNodeCount, allocator);

            mbvhNode.Dispose();
        }

        /// <summary>
        /// Builds the internal CWBVH given a MBVH.
        /// 
        /// CWBVH_COMPRESSED_TRIS hardcoded as undefined due to
        /// comment in tinyBVH.h "doesn't seem to help on GPU"
        /// </summary>
        void BuildInternalCWBVH(ref UnsafeList<MBVHNode> mbvhNode, ref uint mbvhNodeCount, Allocator allocator)
        {
            uint spaceNeeded = bvh.TrisCount * 2 * 5;
            bvh8Data = new UnsafeList<float4>((int)spaceNeeded, allocator, NativeArrayOptions.UninitializedMemory);
            bvh8Tris = new UnsafeList<float4>((int)bvh.PrimCount * 4, allocator, NativeArrayOptions.UninitializedMemory);

            var stackNodePtr = stackalloc MBVHNode*[256];
            var stackNodeAddr = stackalloc uint[256];
            var cost = stackalloc float[8 * 8];
            var assignment = stackalloc int[8];
            var isSlotEmpty = stackalloc bool[8];
            var exyzAndimask = stackalloc byte[4];
            var stackPtr = 1;
            uint nodeDataPtr = 5, triDataPtr = 0;
            stackNodePtr[0] = mbvhNode.GetPointerAt(0);
            stackNodeAddr[0] = 0;

            // Start conversion
            while (stackPtr > 0)
            {
                MBVHNode* orig = stackNodePtr[--stackPtr];
                int currentNodeAddr = (int)stackNodeAddr[stackPtr];
                float3 nodeLo = orig->aabbMin;
                float3 nodeHi = orig->aabbMax;

                // Greedy child node ordering
                float3 nodeCentroid = (nodeLo + nodeHi) * 0.5f;

                for (int s = 0; s < 8; s++)
                {
                    isSlotEmpty[s] = true;
                    assignment[s] = -1;
                    float3 ds = float3(
                        (((s >> 2) & 1) == 1) ? -1.0f : 1.0f,
                        (((s >> 1) & 1) == 1) ? -1.0f : 1.0f,
                        (((s >> 0) & 1) == 1) ? -1.0f : 1.0f
                    );
                    for (int i = 0; i < 8; i++)
                    {
                        if (orig->child[i] == 0) 
                        {
                            cost[CIndex(s, i)] = BVH_FAR;
                        } 
                        else 
                        {
                            MBVHNode* child = mbvhNode.GetPointerAt((int)orig->child[i]);
                            if (child->triCount > 3) // Must be a leaf
                            {
                                SplitMBVHLeaf(ref mbvhNode, ref mbvhNodeCount, orig->child[i], 3);
                            }
                            float3 childCentroid = (child->aabbMin + child->aabbMax) * 0.5f;
                            cost[CIndex(s, i)] = dot(childCentroid - nodeCentroid, ds);
                        }
                    }
                }
                while (true)
                {
                    float minCost = BVH_FAR;
                    int minEntryx = -1, minEntryy = -1;
                    for (int s = 0; s < 8; s++) 
                    {
                        for (int i = 0; i < 8; i++) 
                        {
                            if (assignment[i] == -1 && isSlotEmpty[s] && cost[CIndex(s, i)] < minCost)
                            {
                                minCost = cost[CIndex(s, i)];
                                minEntryx = s;
                                minEntryy = i;
                            }
                        }
                    }
                    if (minEntryx == -1 && minEntryy == -1) break;
                    isSlotEmpty[minEntryx] = false;
                    assignment[minEntryy] = minEntryx;
                }
                for (int i = 0; i < 8; i++) 
                {
                    if (assignment[i] == -1) 
                    {
                        for (int s = 0; s < 8; s++) 
                        {
                            if (isSlotEmpty[s])
                            {
                                isSlotEmpty[s] = false;
                                assignment[i] = s;
                                break;
                            }
                        }
                    }
                }
                MBVHNode* oldNode = orig;
                for (int i = 0; i < 8; i++) 
                {
                    orig->child[assignment[i]] = oldNode->child[i];
                }

                // Calculate quantization parameters for each axis
                int ex = (sbyte)ceil(log2((nodeHi.x - nodeLo.x) / 255.0f));
                int ey = (sbyte)ceil(log2((nodeHi.y - nodeLo.y) / 255.0f));
                int ez = (sbyte)ceil(log2((nodeHi.z - nodeLo.z) / 255.0f));

                // Encode output
                int internalChildCount = 0;
                int leafChildTriCount = 0;
                int childBaseIndex = 0;
                int triangleBaseIndex = 0;
                byte imask = 0;
                for (int i = 0; i < 8; i++)
                {
                    if (orig->child[i] == 0) continue;
                    MBVHNode* child = mbvhNode.GetPointerAt(orig->child[i]);
                    int qlox = (int)floor((child->aabbMin.x - nodeLo.x) / pow(2, ex));
                    int qloy = (int)floor((child->aabbMin.y - nodeLo.y) / pow(2, ey));
                    int qloz = (int)floor((child->aabbMin.z - nodeLo.z) / pow(2, ez));
                    int qhix = (int)ceil((child->aabbMax.x - nodeLo.x) / pow(2, ex));
                    int qhiy = (int)ceil((child->aabbMax.y - nodeLo.y) / pow(2, ey));
                    int qhiz = (int)ceil((child->aabbMax.z - nodeLo.z) / pow(2, ez));
                    byte* baseAddr = (byte*)bvh8Data.GetPointerAt(currentNodeAddr + 2);
                    baseAddr[i + 0] = (byte)qlox;
                    baseAddr[i + 24] = (byte)qhix;
                    baseAddr[i + 8] = (byte)qloy;
                    baseAddr[i + 32] = (byte)qhiy;
                    baseAddr[i + 16] = (byte)qloz;
                    baseAddr[i + 40] = (byte)qhiz;
                    if (!child->IsLeaf)
                    {
                        // Interior node, set params and push onto stack
                        int childNodeAddr = (int)nodeDataPtr;
                        if (internalChildCount++ == 0) 
                        {
                            childBaseIndex = childNodeAddr / 5;
                        }
                        nodeDataPtr += 5;
                        imask = (byte)(imask | 1 << i);

                        // Set the meta field - This calculation assumes children are stored contiguously.
                        byte* childMetaField2 = ((byte*)bvh8Data.GetPointerAt(currentNodeAddr + 1)) + 8;
                        childMetaField2[i] = (byte)((1 << 5) | (24 + (byte)i)); // (tinyBVH.h) I don't see how this accounts for empty children?
                        stackNodePtr[stackPtr] = child;
                        stackNodeAddr[stackPtr++] = (uint)childNodeAddr; // Counted in float4s
                        internalChildCount++;
                        continue;
                    }

                    // Leaf node
                    uint tcount = min(child->triCount, 3u); // (tinyBVH.h): Ensure that's the case. Clamping for now.
                    if (leafChildTriCount == 0) 
                    {
                        triangleBaseIndex = (int)triDataPtr;
                    }
                    int unaryEncodedTriCount = tcount == 1 ? 0b001 : tcount == 2 ? 0b011 : 0b111;

                    // Set the meta field. This calculation assumes children are stored contiguously.
                    byte* childMetaField = ((byte*)bvh8Data.GetPointerAt(currentNodeAddr + 1)) + 8;
                    childMetaField[i] = (byte)((unaryEncodedTriCount << 5) | leafChildTriCount);
                    leafChildTriCount += (int)tcount;
                    for (uint j = 0; j < tcount; j++)
                    {
                        uint triIdx = bvh.primitives[(int)(child->firstTri + j)];
                        uint ti0, ti1, ti2;
                        if (bvh.IsIndexed)
                        {
                            ti0 = bvh.indices[triIdx * 3];
                            ti1 = bvh.indices[triIdx * 3 + 1];
					        ti2 = bvh.indices[triIdx * 3 + 2];
                        }
                        else
                        {
                            ti0 = triIdx * 3;
                            ti1 = triIdx * 3 + 1;
                            ti2 = triIdx * 3 + 2;
                        }

                        float4 t = float4(bvh.vertices[ti0], 0);
                        bvh8Tris[(int)triDataPtr + 0] = float4(bvh.vertices[ti2] - t.xyz, 0);
                        bvh8Tris[(int)triDataPtr + 1] = float4(bvh.vertices[ti1] - t.xyz, 0);
                        t.w = *(float*)&triIdx;
                        bvh8Tris[(int)triDataPtr + 2] = t;
                        triDataPtr += 3;
                    }
                }
                exyzAndimask[0] = *(byte*)&ex;
                exyzAndimask[1] = *(byte*)&ey;
                exyzAndimask[2] = *(byte*)&ez;
                exyzAndimask[3] = imask;
                bvh8Data[currentNodeAddr + 0] = float4(nodeLo, *(float*)&exyzAndimask);
                var bvh8Data_1 = bvh8Data[currentNodeAddr + 1];
                bvh8Data_1.x = *(float*)&childBaseIndex;
                bvh8Data_1.y = *(float*)&triangleBaseIndex;
                bvh8Data[currentNodeAddr + 1] = bvh8Data_1;
            }
        }

        /// <summary>
        /// Builds the internal MBVH.
        /// M is hardcoded as 8 in this case.
        /// </summary>
        void BuildInternalMBVH (out UnsafeList<MBVHNode> mbvhNode, out uint mbvhNodeCount)
        {
            // MVH8
            uint spaceNeeded = bvh.NodeCount;
            spaceNeeded += bvh.NodeCount >> 1;

            mbvhNodeCount = 0;
            mbvhNode = new UnsafeList<MBVHNode>((int)spaceNeeded,
                Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            mbvhNode.Length = mbvhNode.Capacity;

            // Create a MBVH node for each BVH2 node
            for (uint i = 0; i < bvh.NodeCount; i++)
            {
                BVHNode* orig = bvh.bvhNode.GetPointerAt(i);
                var node = new MBVHNode
                {
                    aabbMin = orig->aabbMin,
                    aabbMax = orig->aabbMax
                };
                if (orig->IsLeaf)
                {
                    node.triCount = orig->triCount;
                    node.firstTri = orig->leftFirst;
                }
                else
                {
                    node.child[0] = orig->leftFirst;
                    node.child[1] = orig->leftFirst + 1;
                    node.childCount = 2;
                }
                mbvhNode[(int)i] = node;
            }

            // Collapse the whole tree
            var stack = stackalloc uint[128];
            uint stackPtr = 0, nodeIdx = 0;
            while (true)
            {
                MBVHNode* node = mbvhNode.GetPointerAt(nodeIdx);
                while (node->childCount < 8)
                {
                    int bestChild = -1;
                    float bestChildSA = 0;
                    for (int i = 0; i < node->childCount; i++)
                    {
                        // See if we can adopt child i
                        MBVHNode* child = mbvhNode.GetPointerAt(node->child[i]);
                        if (!child->IsLeaf && node->childCount - 1 + child->childCount <= 8)
                        {
                            float childSA = SA(child->aabbMin, child->aabbMax);
                            if (childSA > bestChildSA)
                            {
                                bestChild = i;
                                bestChildSA = childSA;
                            }
                        }
                    }
                    if (bestChild == -1) break; // Could not adopt
                    MBVHNode* child2 = mbvhNode.GetPointerAt(node->child[bestChild]);
                    node->child[bestChild] = child2->child[0];
                    for (int i = 0; child2->childCount > 0; i++)
                    {
                        node->child[node->childCount++] = child2->child[i];
                    }
                }

                // Done with the node. Proceed with the children.
                for (int i = 0; i < node->childCount; i++)
                {
                    uint childIdx = node->child[i];
                    MBVHNode* child3 = mbvhNode.GetPointerAt(childIdx);
                    if (!child3->IsLeaf)
                    {
                        stack[stackPtr++] = childIdx;
                    }
                }
                if (stackPtr == 0) break;
                nodeIdx = stack[--stackPtr];
            }

            // Special case where the root is a leaf. Add extra level (CWBVH needs this)
            MBVHNode* root = mbvhNode.GetPointerAt(0);
            if (root->IsLeaf)
            {
                mbvhNode[1] = new MBVHNode
                {
                    aabbMin = root->aabbMin,
                    aabbMax = root->aabbMax,
                    childCount = 0,
                    firstTri = root->firstTri,
                    triCount = root->triCount
                };
                root->childCount = 1;
                root->child[0] = 1;
                root->triCount = 0;

                mbvhNodeCount = 2;
            }
            else
            {
                mbvhNodeCount = bvh.NodeCount;
            }
        }

        /// <summary>
        /// CWBVH requires that a leaf has no more than 3 primitives,
        /// but regular BVH construction does not guarantee this. So, here we split
        /// busy leafs recursively in multiple leaves, until the requirement is met.
        /// </summary>
        void SplitMBVHLeaf(ref UnsafeList<MBVHNode> mbvhNode, ref uint mbvhNodeCount, uint nodeIdx, uint maxPrims = 3)
        {
            uint* primIdx = bvh.primitives.Ptr;
            Fragment* fragment = bvh.fragments.Ptr;
            MBVHNode* node = mbvhNode.GetPointerAt(nodeIdx);

            // This also caches interior nodes (if a child has above 3 primitives)
            if (node->triCount <= maxPrims) return;

            // Place all primitives in a new node and make this the first child of 'node'
            node->child[0] = mbvhNodeCount;
            mbvhNodeCount++;
            MBVHNode* firstChild = mbvhNode.GetPointerAt((int)mbvhNodeCount);
	        firstChild->triCount = node->triCount;
	        firstChild->firstTri = node->firstTri;
	        uint nextChild = 1;

	        // Share with new sibling nodes
	        while (firstChild->triCount > maxPrims && nextChild < 8)
	        {
                node->child[nextChild] = mbvhNodeCount;
                mbvhNodeCount++;
                MBVHNode* child = mbvhNode.GetPointerAt(mbvhNodeCount);
                firstChild->triCount -= maxPrims; 
                child->triCount = maxPrims;
		        child->firstTri = firstChild->firstTri + firstChild->triCount;

		        nextChild++;
	        }
	        for (uint i = 0; i < nextChild; i++)
	        {
                // NOTE (tinyBVH.h): Why is this producing wrong aabbs for SBVH?
                // NOTE (unityBVH): Assumed MBVH as refittable.
		        MBVHNode* child = mbvhNode.GetPointerAt(node->child[i]);
                child->aabbMin = float3(BVH_FAR);
                child->aabbMax = float3(-BVH_FAR);
                for (uint fi, j = 0; j < child->triCount; j++) {
                    fi = primIdx[child->firstTri + j];
				    child->aabbMin = min(child->aabbMin, fragment[fi].bmin);
				    child->aabbMax = max(child->aabbMax, fragment[fi].bmax);
                }
            }
	        node->triCount = 0;

            // Recurse. Should be rare
            if (firstChild->triCount > maxPrims) {
                SplitMBVHLeaf(ref mbvhNode, ref mbvhNodeCount, node->child[0], maxPrims);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int CIndex(int s, int i) => s * 8 + i;
    }
}