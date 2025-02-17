using UnityEngine;

namespace DDGIURP
{
    public static class ShaderIds
    {
        public static readonly int
            irrRes = Shader.PropertyToID("irrRes"),
            visRes = Shader.PropertyToID("visRes"),
            irrResPad = Shader.PropertyToID("irrResPad"),
            visResPad = Shader.PropertyToID("visResPad"),
            probeDimensions = Shader.PropertyToID("probeDimensions"),
            _Visibility = Shader.PropertyToID("_Visibility"),
            _Irradiance = Shader.PropertyToID("_Irradiance"),
            _probeData = Shader.PropertyToID("_probeData"),
            VertexBuffer = Shader.PropertyToID("VertexBuffer"),
            IndexBuffer = Shader.PropertyToID("IndexBuffer"),
            VertexPositionBuffer = Shader.PropertyToID("VertexPositionBuffer"),
            TriangleAttributesBuffer = Shader.PropertyToID("TriangleAttributesBuffer"),
            VertexStride = Shader.PropertyToID("VertexStride"),
            PositionOffset = Shader.PropertyToID("PositionOffset"),
            NormalOffset = Shader.PropertyToID("NormalOffset"),
            UVOffset = Shader.PropertyToID("UVOffset"),
            _VoxelTarget = Shader.PropertyToID("_VoxelTarget"),
            resolution = Shader.PropertyToID("resolution"),
            ViewProjectionMatrix = Shader.PropertyToID("ViewProjectionMatrix"),
            unity_ObjectToWorld = Shader.PropertyToID("unity_ObjectToWorld"),
            _ObjectToWorld = Shader.PropertyToID("_ObjectToWorld"),
            VP_MATRIX = Shader.PropertyToID("VP_MATRIX"),
            invScale = Shader.PropertyToID("invScale"),
            VolumeCenter = Shader.PropertyToID("VolumeCenter"),
            _BaseMap = Shader.PropertyToID("_BaseMap"),
            _BaseMap_ST = Shader.PropertyToID("_BaseMap_ST"),
            _ = Shader.PropertyToID("_");
    }
}