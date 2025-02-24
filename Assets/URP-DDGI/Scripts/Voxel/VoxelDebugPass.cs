using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEngine.Rendering.Universal;

namespace DDGIURP
{
    public class VoxelDebugPass : ScriptableRenderPass
    {
        internal DDGISettings settings;
        private Material material;
        private Shader shader;

        class PassData
        {
            public DDGISettings settings;
            public Material material;
            public Matrix4x4 viewProjectionMatrix;
            public Vector3 camWorldPos;
        }

        public VoxelDebugPass (DDGISettings settings)
        {
            renderPassEvent = RenderPassEvent.BeforeRenderingPostProcessing;

            this.settings = settings;
            shader = Resources.Load<Shader>("Shaders/VoxelRaymarchDebug");
            material = new Material(shader);
        }

        public void Dispose ()
        {
            Object.DestroyImmediate(material);
        }

        

        public override void RecordRenderGraph (RenderGraph renderGraph, ContextContainer frameData)
        {
            var cameraData = frameData.Get<UniversalCameraData>();
            var resourceData = frameData.Get<UniversalResourceData>();

            if (cameraData.isPreviewCamera) return;
            if (resourceData.isActiveTargetBackBuffer) return;

            var cameraToWorldMatrix = cameraData.camera.cameraToWorldMatrix;
            var projectionMatrix = GL.GetGPUProjectionMatrix(cameraData.camera.projectionMatrix, true);
            var viewProjectionMatrix = projectionMatrix * cameraToWorldMatrix.inverse;

            var source = resourceData.activeColorTexture;
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Debug Voxel", out var passData))
            {
                builder.AllowPassCulling(false);
                builder.SetRenderAttachment(source, 0);
                builder.UseGlobalTexture(ShaderIds._VoxelCascade0, AccessFlags.Read);
                passData.material = material;
                passData.viewProjectionMatrix = viewProjectionMatrix;
                passData.camWorldPos = cameraData.camera.transform.position;
                passData.settings = settings;
                builder.SetRenderFunc<PassData>(ExecutePass);
            }
        }

        static void ExecutePass (PassData passData, RasterGraphContext context)
        {
            var cmd = context.cmd;
            var mat = passData.material;
            var viewProjectionMatrix = passData.viewProjectionMatrix;

            float scale = passData.settings.voxelBaseCascadeScale;
            Vector3 center = math.round(passData.camWorldPos / scale) * scale;

            mat.SetMatrix(ShaderIds.VP_MATRIX, viewProjectionMatrix);
            mat.SetMatrix(ShaderIds.INV_VP_MATRIX, viewProjectionMatrix.inverse);
            mat.SetVector(ShaderIds._BlitScaleBias, new Vector4(1, 1, 0, 0));
            mat.SetVector(ShaderIds._CameraWorldPos, passData.camWorldPos);
            mat.SetVector(ShaderIds._VoxelCascadeO_Origin, center);

            /*uniform float4 _BlitScaleBias;
            uniform float4 _BlitScaleBiasRt;
            uniform float4 _BlitTexture_TexelSize;
            uniform float _BlitMipLevel;
            uniform float2 _BlitTextureSize;
            uniform uint _BlitPaddingSize;
            uniform int _BlitTexArraySlice;
            uniform float4 _BlitDecodeInstructions;*/


            cmd.DrawProcedural(Matrix4x4.identity, mat, 0, MeshTopology.Triangles, 3);
        }
    }
}