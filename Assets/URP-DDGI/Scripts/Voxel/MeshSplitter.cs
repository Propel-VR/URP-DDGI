using UnityEngine;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Rendering;

namespace DDGIURP
{
    // TODO: Skinned mesh support
    // TODO: Optimize
    [BurstCompile]
    public static class MeshSplitter
    {
        static readonly VertexAttributeDescriptor[] defaultMeshAttributes = new[]
        {
            new VertexAttributeDescriptor(VertexAttribute.Position, dimension: 3, stream: 0),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord0, dimension: 2, stream: 1),
            //new VertexAttributeDescriptor(VertexAttribute.Normal, dimension: 3, stream: 2),
        };

        public static Mesh CreateSplitMesh (Mesh originalMesh, bool isSkinned)
        {
            int subMeshCount = originalMesh.subMeshCount;
            int indexCount = 0;
            for(int i = 0; i < subMeshCount; i++)
            {
                indexCount += originalMesh.GetSubMesh(i).indexCount;
            }

            var copyMeshArray = Mesh.AllocateWritableMeshData(1);
            var copyMeshData = copyMeshArray[0];
            using (var originalMeshArray = Mesh.AcquireReadOnlyMeshData(originalMesh))
            {
                var originalMeshData = originalMeshArray[0];

                var originalPosition = new NativeArray<Vector3>(originalMeshData.vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                var originalUV0 = new NativeArray<Vector2>(originalMesh.vertexCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                originalMeshData.GetVertices(originalPosition);
                originalMeshData.GetUVs(0, originalUV0);
                var originalIndex = originalMeshData.GetIndexData<ushort>();

                copyMeshData.SetVertexBufferParams(indexCount, defaultMeshAttributes);
                copyMeshData.SetIndexBufferParams(indexCount, originalMesh.indexFormat);
                var copyIndexArray = copyMeshData.GetIndexData<ushort>();
                var copyPosition = copyMeshData.GetVertexData<float3>(stream: 0);
                var copyUV0 = copyMeshData.GetVertexData<float2>(stream: 1);

                for (int i = 0; i < copyIndexArray.Length; i++)
                {
                    copyIndexArray[i] = (ushort)i;
                    int index = originalIndex[i];
                    copyPosition[i] = originalPosition[index];
                    copyUV0[i] = originalUV0[index];
                }

                copyMeshData.subMeshCount = subMeshCount;
                indexCount = 0;
                for (int i = 0; i < subMeshCount; i++)
                {
                    var originalSubmesh = originalMeshData.GetSubMesh(i);
                    var copySubmesh = originalSubmesh;
                    copySubmesh.indexStart = indexCount;
                    copySubmesh.vertexCount = originalSubmesh.indexCount;
                    copySubmesh.firstVertex = indexCount;
                    copySubmesh.baseVertex = 0;
                    indexCount += originalSubmesh.indexCount;

                    copyMeshData.SetSubMesh(i, copySubmesh);
                }

                originalPosition.Dispose();
                originalUV0.Dispose();
            }

            // Create the new mesh
            Mesh copyMesh = new Mesh();
            copyMesh.name = originalMesh.name + " (Copy)";
            Mesh.ApplyAndDisposeWritableMeshData(copyMeshArray, copyMesh);
            copyMesh.RecalculateBounds();
            if(isSkinned) copyMesh.MarkDynamic();
            copyMesh.RecalculateNormals();

            return copyMesh;
        }
    }
}