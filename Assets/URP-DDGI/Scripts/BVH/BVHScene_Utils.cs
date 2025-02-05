using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

namespace DDGIURP
{
    public static class BVHScene_Utils
    {
        // Finds the offset of an attribute as well as the vertex stride.
        public static void FindVertexAttribute(Mesh mesh, VertexAttribute targetAttribute, out int attributeOffset, out int vertexStride)
        {
            VertexAttributeDescriptor[] vertexAttributes = mesh.GetVertexAttributes();
            attributeOffset = 0;
            vertexStride = 0;

            foreach (var attribute in vertexAttributes)
            {
                if (attribute.attribute == targetAttribute)
                {
                    attributeOffset = vertexStride;
                }

                // Increment vertexStride by the size of the current attribute
                switch (attribute.format)
                {
                    case VertexAttributeFormat.Float32:
                    case VertexAttributeFormat.UInt32:
                    case VertexAttributeFormat.SInt32:
                        vertexStride += 4 * attribute.dimension;
                        break;
                    case VertexAttributeFormat.Float16:
                    case VertexAttributeFormat.UNorm16:
                    case VertexAttributeFormat.SNorm16:
                        vertexStride += 2 * attribute.dimension;
                        break;
                    case VertexAttributeFormat.UNorm8:
                    case VertexAttributeFormat.SNorm8:
                    case VertexAttributeFormat.UInt8:
                    case VertexAttributeFormat.SInt8:
                        vertexStride += 1 * attribute.dimension;
                        break;
                    default:
                        Debug.LogError("Unsupported vertex format");
                        break;
                }
            }
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