using CommonVars;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEngine.Rendering.Universal;

namespace DDGIURP
{
    public class VoxelDebugPass : ScriptableRenderPass
    {
        private Material material;
        private Shader shader;

        public VoxelDebugPass ()
        {
            renderPassEvent = RenderPassEvent.AfterRenderingPostProcessing;

            shader = Resources.Load<Shader>("Shaders/VoxelRaytraceDebug");
            material = new Material(shader);
        }

        public void SetVoxelTarget(RenderTexture voxelTarget, int resolution)
        {
            material.SetTexture(ShaderIds._VoxelTarget, voxelTarget);
            material.SetInt(ShaderIds.resolution, resolution);
        }

        ~VoxelDebugPass ()
        {
            Object.DestroyImmediate(material);
        }

        class PassData
        {

        }

        public override void RecordRenderGraph (RenderGraph renderGraph, ContextContainer frameData)
        {
            var cameraData = frameData.Get<UniversalCameraData>();
            var resourceData = frameData.Get<UniversalResourceData>();

            if (cameraData.isPreviewCamera) return;
            if (!resourceData.activeColorTexture.IsValid()) return;

            var cameraToWorldMatrix = cameraData.camera.cameraToWorldMatrix;
            var projectionMatrix = GL.GetGPUProjectionMatrix(cameraData.camera.projectionMatrix, true);
            material.SetMatrix(ShaderIds.VP_MATRIX, projectionMatrix * cameraToWorldMatrix.inverse);

            var blitParams = new RenderGraphUtils.BlitMaterialParameters() {
                material = material,
                destination = resourceData.activeColorTexture
            };
            renderGraph.AddBlitPass(blitParams);
        }
    }
}