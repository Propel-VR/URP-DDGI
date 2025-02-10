using EasyButtons;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using Unity.Jobs;


namespace DDGIURP
{
    /// <summary>
    /// Root of the BVH scene. Ensure it gets generated and continuously updated.
    /// Automatic BLAS and TLAS representations.
    /// 
    /// Based on unity-TinyBVH and TruePathTracer.
    /// </summary>
    [ExecuteAlways]
    public class BVHScene : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] List<Renderer> filteredRenderers = new();

        [Header("Debug")]
        [ReadOnly, SerializeField] int totalTrackedMeshes;
        [ReadOnly, SerializeField] int totalUniqueMeshes;
        [ReadOnly, SerializeField] int totalVertexCount;
        [ReadOnly, SerializeField] int totalTriangleCount;
        [SerializeField] List<BLAS> BLAS;

        const int VERTEX_SIZE = 3 * sizeof(float);
        const int TRIS_ATTRIBUTE_SIZE = 15 * sizeof(float);
        const int TRIS_PACKED_ATTRIBUTE_SIZE = 12 * sizeof(float);

        private ComputeShader meshProcessingShader;
        private LocalKeyword has32BitIndicesKeyword;
        private LocalKeyword hasNormalsKeyword;
        private LocalKeyword hasUVsKeyword;

        private ComputeBuffer vertexPositionBuffer;
        private ComputeBuffer triangleAttributeBuffer;
        private NativeArray<float3> vertexPositionBufferCPU;

        private HashSet<Mesh> uniqueMeshes;
        private DateTime readbackStartTime;

        void OnDisable () => ClearBLAS();

        private void ClearBLAS ()
        {
            vertexPositionBuffer?.Release();
            triangleAttributeBuffer?.Release();
            if (vertexPositionBufferCPU.IsCreated) vertexPositionBufferCPU.Dispose();
            foreach (var blas in BLAS)
            {
                if (blas.bvh.IsCreated) blas.bvh.Dispose();
                blas.bvh = default;
            }
            BLAS.Clear();
        }

        private void OnDrawGizmosSelected()
        {
            if (BLAS == null) return;
            foreach (var blas in BLAS)
            {
                if (!blas.bvh.IsCreated) return;

                Gizmos.color = Color.red;
                for(int i = 0; i < blas.bvh.NodeCount; i++)
                {
                    var node = blas.bvh.bvhNode[i];
                    var center = (node.aabbMin + node.aabbMax) * 0.5f;
                    var size = math.abs(node.aabbMax - node.aabbMin);
                    Gizmos.DrawWireCube(center, size);
                }
            }
        }



        // TODO: How to automatically detect rendereres without requiring a script on them, or a references to them?
        // We want to ensure raytracing is enabled for them. Maybe filter with RenderLayers?
        [Button]
        public void GetAllRenderers ()
        {
            filteredRenderers = FindObjectsByType<Renderer>(FindObjectsSortMode.None)
                .Where((r) => r.rayTracingMode != RayTracingMode.Off).ToList();
            totalTrackedMeshes = filteredRenderers.Count;
        }

#if UNITY_EDITOR
        [Button]
        public void ProcessBLAS ()
        {
            // Load compute shader, clear previous
            meshProcessingShader = Resources.Load<ComputeShader>("Shaders/MeshProcessing");
            has32BitIndicesKeyword = meshProcessingShader.keywordSpace.FindKeyword("HAS_32_BIT_INDICES");
            hasNormalsKeyword = meshProcessingShader.keywordSpace.FindKeyword("HAS_NORMALS");
            hasUVsKeyword = meshProcessingShader.keywordSpace.FindKeyword("HAS_UVS");
            ClearBLAS();

            if(UnityEditor.EditorApplication.isPlaying)
            {
                Debug.LogError("[DDGI Scene] Leave playmode to build BLAS.");
                return;
            }
            Debug.Log("[DDGI Scene] Building BLAS. Do not enter playmode.");

            // Gather all the meshes to build a BLAS out of them
            BLAS = new List<BLAS>();
            uniqueMeshes = new HashSet<Mesh>();
            foreach (var renderer in filteredRenderers)
            {
                if(renderer.gameObject.TryGetComponent<MeshFilter>(out var meshFilter) && meshFilter.sharedMesh != null)
                {
                    uniqueMeshes.Add(meshFilter.sharedMesh);
                }
                if(renderer.gameObject.TryGetComponent<SkinnedMeshRenderer>(out var skinnedRenderer) && skinnedRenderer.sharedMesh != null)
                {
                    uniqueMeshes.Add(skinnedRenderer.sharedMesh);
                }
            }
            totalUniqueMeshes = uniqueMeshes.Count;

            // Gather total vertex count
            totalVertexCount = 0;
            totalTriangleCount = 0;
            int meshIndex = 0;
            foreach (var mesh in uniqueMeshes)
            {
                var triangleCount = BVHScene_Utils.GetTriangleCount(mesh);
                BLAS.Add(new BLAS(mesh, meshIndex++, totalTriangleCount, triangleCount));
                totalVertexCount += mesh.vertexCount;
                totalTriangleCount += BVHScene_Utils.GetTriangleCount(mesh);
            }

            vertexPositionBuffer = new ComputeBuffer(totalTriangleCount * 3, VERTEX_SIZE);
            vertexPositionBufferCPU = new NativeArray<float3>(totalTriangleCount * 3, Allocator.Persistent);
            triangleAttributeBuffer = new ComputeBuffer(totalTriangleCount, TRIS_PACKED_ATTRIBUTE_SIZE);

            // Downloading mesh to be processed
            Debug.Log("[DDGI Scene] Coping meshes to temporarly buffer");
            int i = 0;
            totalTriangleCount = 0;
            foreach (var mesh in uniqueMeshes)
            {
                // Ensure meshes can be downloaded
                mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
                mesh.indexBufferTarget |= GraphicsBuffer.Target.Raw;

                var vertexBuffer = mesh.GetVertexBuffer(0);
                var indexBuffer = mesh.GetIndexBuffer();
                var triangleCount = BVHScene_Utils.GetTriangleCount(mesh);

                // Determine where in the Unity vertex buffer each vertex attribute is
                BVHScene_Utils.FindVertexAttribute(mesh, VertexAttribute.Position, out int positionOffset, out int vertexStride);
                BVHScene_Utils.FindVertexAttribute(mesh, VertexAttribute.Normal, out int normalOffset, out vertexStride);
                BVHScene_Utils.FindVertexAttribute(mesh, VertexAttribute.TexCoord0, out int uvOffset, out vertexStride);

                // Configure compute shader
                meshProcessingShader.SetBuffer(0, "VertexBuffer", vertexBuffer);
                meshProcessingShader.SetBuffer(0, "IndexBuffer", indexBuffer);
                meshProcessingShader.SetBuffer(0, "VertexPositionBuffer", vertexPositionBuffer);
                meshProcessingShader.SetBuffer(0, "TriangleAttributesBuffer", triangleAttributeBuffer);
                meshProcessingShader.SetInt("VertexStride", vertexStride);
                meshProcessingShader.SetInt("PositionOffset", positionOffset);
                meshProcessingShader.SetInt("NormalOffset", normalOffset);
                meshProcessingShader.SetInt("UVOffset", uvOffset);
                meshProcessingShader.SetInt("TriangleCount", triangleCount);
                meshProcessingShader.SetInt("OutputTriangleStart", totalTriangleCount);

                // Set keywords based on format/attributes of this mesh
                meshProcessingShader.SetKeyword(has32BitIndicesKeyword, (mesh.indexFormat == IndexFormat.UInt32));
                meshProcessingShader.SetKeyword(hasNormalsKeyword, mesh.HasVertexAttribute(VertexAttribute.Normal));
                meshProcessingShader.SetKeyword(hasUVsKeyword, mesh.HasVertexAttribute(VertexAttribute.TexCoord0));

                totalTriangleCount += triangleCount;

                meshProcessingShader.Dispatch(0, Mathf.CeilToInt(triangleCount / 64.0f), 1, 1);
                i++;
            }
            Debug.Log("[DDGI Scene] Finished Copy. Starting Readback...");

            readbackStartTime = DateTime.UtcNow;
            AsyncGPUReadback.RequestIntoNativeArray(ref vertexPositionBufferCPU, vertexPositionBuffer, ProcessBLAS2);
        }

        unsafe void ProcessBLAS2 (AsyncGPUReadbackRequest request)
        {
            if (request.hasError)
            {
                Debug.LogError("[DDGI Scene] Failed to readback.");
                return;
            }
            var readbackTime = DateTime.UtcNow - readbackStartTime;
            Debug.Log($"[DDGI Scene] Readback completed in {readbackTime.Milliseconds}ms. Starting BVH generation...");

            var sw = new Stopwatch();
            sw.Start();

            // Allocate the BLAS BVH and generate them in threads.
            // Each unique mesh is calculated in a separate thread.
            var bvhTempArray = new NativeArray<BVH>(BLAS.Count, Allocator.TempJob);
            int i = 0;
            foreach(var blas in BLAS)
            {
                var bvh = new BVH();
                var vertsSlice = vertexPositionBufferCPU.GetSubArray(blas.bufferIndex * 3, blas.bufferCount * 3);
                bvh.Build(vertsSlice, Allocator.Domain);
                bvhTempArray[i] = bvh;
                i++;
            }
            var bvhJob = new BVHJob(bvhTempArray);
            bvhJob.Schedule(bvhTempArray.Length, 1).Complete();
            i = 0;
            foreach (var blas in BLAS)
            {
                blas.bvh = bvhTempArray[i];
                i++;
            }
            bvhTempArray.Dispose();

            sw.Stop();
            Debug.Log($"[DDGI Scene] BVH completed in {sw.Elapsed.TotalMilliseconds}ms.");
        }
    }
#endif
}
