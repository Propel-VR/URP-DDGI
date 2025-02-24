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
            resolution = Shader.PropertyToID("resolution"),
            ViewProjectionMatrix = Shader.PropertyToID("ViewProjectionMatrix"),
            unity_ObjectToWorld = Shader.PropertyToID("unity_ObjectToWorld"),
            _ObjectToWorld = Shader.PropertyToID("_ObjectToWorld"),
            VP_MATRIX = Shader.PropertyToID("VP_MATRIX"),
            INV_VP_MATRIX = Shader.PropertyToID("INV_VP_MATRIX"),
            invScale = Shader.PropertyToID("invScale"),
            VolumeCenter = Shader.PropertyToID("VolumeCenter"),
            _BaseMap = Shader.PropertyToID("_BaseMap"),
            _BaseMap_ST = Shader.PropertyToID("_BaseMap_ST"),
            _VoxelCascade0 = Shader.PropertyToID("_VoxelCascade0"),
            _BlitScaleBias = Shader.PropertyToID("_BlitScaleBias"),
            _BlitScaleBiasRt = Shader.PropertyToID("_BlitScaleBiasRt"),
            _BlitTexture_TexelSize = Shader.PropertyToID("_BlitTexture_TexelSize"),
            _BlitMipLevel = Shader.PropertyToID("_BlitMipLevel"),
            _BlitPaddingSize = Shader.PropertyToID("_BlitPaddingSize"),
            _BlitTexArraySlice = Shader.PropertyToID("_BlitTexArraySlice"),
            _BlitDecodeInstructions = Shader.PropertyToID("_BlitDecodeInstructions"),
            _CameraWorldPos = Shader.PropertyToID("_CameraWorldPos"),
            _VoxelCascadeO_Origin = Shader.PropertyToID("_VoxelCascadeO_Origin"),
            _ = Shader.PropertyToID("_");
    }
}