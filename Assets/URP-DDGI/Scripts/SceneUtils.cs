using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

namespace DDGIURP
{
    public struct AttributesResults
    {
        public int vertexStride;

        public AttributeResult position;
        public AttributeResult uv0;
        public AttributeResult normal;
        public AttributeResult tangent;
    }

    public struct AttributeResult
    {
        public bool exists;
        public int offset;
        public int dimension;
        public int formatSize;
    }

    public static class SceneUtils
    {
        // Finds the offset of an attribute as well as the vertex stride.
        private static List<VertexAttributeDescriptor> tempAttributesDesc = new(64);

        public static AttributesResults FindAttributes (Mesh mesh)
        {
            tempAttributesDesc.Clear();
            mesh.GetVertexAttributes(tempAttributesDesc);

            var results = new AttributesResults();

            int vertexStride = 0;
            int currentFormat = 0;
            foreach (var attDesc in tempAttributesDesc)
            {
                var result = new AttributeResult();
                result.exists = true;
                result.offset = vertexStride;
                result.dimension = attDesc.dimension;

                // Increment vertexStride by the size of the current attribute
                currentFormat = attDesc.format switch
                {
                    VertexAttributeFormat.Float32 or VertexAttributeFormat.UInt32 or VertexAttributeFormat.SInt32 => 4,
                    VertexAttributeFormat.Float16 or VertexAttributeFormat.UNorm16 or VertexAttributeFormat.SNorm16 => 2,
                    VertexAttributeFormat.UNorm8 or VertexAttributeFormat.SNorm8 or VertexAttributeFormat.UInt8 or VertexAttributeFormat.SInt8 => 1,
                    _ => 0,
                };
                vertexStride += currentFormat * attDesc.dimension;
                result.formatSize = currentFormat;

                // Set result;
                switch (attDesc.attribute)
                {
                    case VertexAttribute.Position:
                    results.position = result;
                    break;
                    case VertexAttribute.TexCoord0:
                    results.uv0 = result;
                    break;
                    case VertexAttribute.Tangent:
                    results.tangent = result;
                    break;
                    case VertexAttribute.Normal:
                    results.normal = result;
                    break;
                }
            }

            results.vertexStride = vertexStride;
            return results;
        }

        // Returns the total number of triangles in a given mesh.
        public static int GetTriangleCount(Mesh mesh)
        {
            int result = 0;
            for (int i = 0; i < mesh.subMeshCount; ++i)
            {
                result += (int)mesh.GetIndexCount(i) / 3;
            }
            return result;
        }

    }
}