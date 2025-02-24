using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEngine.Rendering.RendererUtils;
using Unity.Mathematics;

namespace DDGIURP
{
    public class VoxelizePass : ScriptableRenderPass, System.IDisposable
    {
        internal DDGISettings settings;
        internal RTHandle voxel0Volume;
        internal RTHandle voxelTargetColor;
        internal RTHandle voxelTargetDepth;

        private RendererListHandle rendererList0;
        private ComputeShader voxelCompute;
        private int voxelKernel;
        private Camera cascade0Camera;

        internal class PassData
        {
            public DDGISettings settings;
            public RendererListHandle rendererList;
            public ComputeShader voxelCompute;
            public int voxelKernel;
            public TextureHandle voxel0Handle;
            public Vector3 cameraCenter;
        }

        List<ShaderTagId> shaderTagList = new()
        {
            new ShaderTagId ("UniversalForward"),
            new ShaderTagId ("SRPDefaultUnlit"),
            new ShaderTagId ("UniversalForwardOnly")
        };

        public VoxelizePass (DDGISettings settings)
        {
            this.renderPassEvent = RenderPassEvent.AfterRenderingShadows;

            this.settings = settings;
            FreeResources();
            AllocateResources();
        }

        public void SetCamera (Camera camera) => cascade0Camera = camera;

        public void Dispose ()
        {
            FreeResources();
        }

        private void AllocateResources ()
        {
            var voxelTargetDescColor = new RenderTextureDescriptor()
            {
                dimension = TextureDimension.Tex2D,
                width = settings.voxelCascadeResolution,
                height = settings.voxelCascadeResolution,
                volumeDepth = 1,
                graphicsFormat = GraphicsFormat.R8_UNorm,
                useMipMap = false,
                msaaSamples = 1,
            };
            var voxelTargetDescDepth = voxelTargetDescColor;
            voxelTargetDescDepth.graphicsFormat = GraphicsFormat.None;
            voxelTargetDescDepth.depthStencilFormat = GraphicsFormat.D16_UNorm;

            var voxelVolumeDesc = new RenderTextureDescriptor()
            {
                dimension = TextureDimension.Tex3D,
                width = settings.voxelCascadeResolution,
                height = settings.voxelCascadeResolution,
                volumeDepth = settings.voxelCascadeResolution,
                graphicsFormat = GraphicsFormat.R8G8B8A8_UNorm,
                enableRandomWrite = true,
                msaaSamples = 1,
            };
            

            voxel0Volume = RTHandles.Alloc(voxelVolumeDesc, name: "Voxel0CascadeVolume");
            voxelTargetColor = RTHandles.Alloc(voxelTargetDescColor, name: "VoxelRenderTargetColor");
            voxelTargetDepth = RTHandles.Alloc(voxelTargetDescDepth, name: "VoxelRenderTargetDepth");

            voxelCompute = Resources.Load<ComputeShader>("Shaders/Voxel");
            voxelKernel = voxelCompute.FindKernel("Clear");
        }

        void FreeResources ()
        {
            if (voxel0Volume != null)
            {
                RTHandles.Release(voxel0Volume);
                voxel0Volume = null;
            }
            if (voxelTargetColor != null)
            {
                RTHandles.Release(voxelTargetColor);
                voxelTargetColor = null;
            }
            if (voxelTargetDepth != null)
            {
                RTHandles.Release(voxelTargetDepth);
                voxelTargetDepth = null;
            }
        }

        void CreateRenderLists (RenderGraph renderGraph, ContextContainer frameData)
        {
            var cameraData = frameData.Get<UniversalCameraData>();
            var renderingData = frameData.Get<UniversalRenderingData>();
            var lightData = frameData.Get<UniversalLightData>();

            var sortingSettings = new SortingSettings()
            {
                cameraPosition = cameraData.worldSpaceCameraPos,
                criteria = SortingCriteria.None,
            };
            var drawSettings = new DrawingSettings(shaderTagList[0], sortingSettings)
            {
                enableDynamicBatching = renderingData.supportsDynamicBatching,
                enableInstancing = true,
                perObjectData = PerObjectData.None,

            };
            var filteringSettings = new FilteringSettings()
            {
                layerMask = 1 << settings.voxelRendererLayer,
                renderingLayerMask = ~0u,
                renderQueueRange = RenderQueueRange.opaque,
            };
            //var renderListParams0 = new RendererListParams(renderingData.cullResults, drawSettings, filteringSettings);
            RendererListDesc desc = new RendererListDesc(shaderTagList[0], renderingData.cullResults, cascade0Camera)
            {
                sortingCriteria = SortingCriteria.None,
                layerMask = 1 << settings.voxelRendererLayer,
                renderingLayerMask = ~0u,
                rendererConfiguration = PerObjectData.None,
                renderQueueRange = RenderQueueRange.opaque,
                stateBlock = new RenderStateBlock(RenderStateMask.Nothing),
                
            };
            //rendererList0 = renderGraph.CreateRendererList(renderListParams0);
            rendererList0 = renderGraph.CreateRendererList(desc);
        }

        public override void RecordRenderGraph (RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();
            var cameraData = frameData.Get<UniversalCameraData>();
            var renderingData = frameData.Get<UniversalRenderingData>();

            CreateRenderLists(renderGraph, frameData);

            var voxel0Handle = renderGraph.ImportTexture(voxel0Volume, new ImportResourceParams() { discardOnLastUse = false, clearOnFirstUse = false });
            var voxelTargetColorHandle = renderGraph.ImportTexture(voxelTargetColor, new ImportResourceParams() { discardOnLastUse = false, clearOnFirstUse = false});
            var voxelTargetDepthHandle = renderGraph.ImportTexture(voxelTargetDepth, new ImportResourceParams() { discardOnLastUse = false, clearOnFirstUse = false});

            using (var builder = renderGraph.AddComputePass("Voxel Cascade 0 Clear", out PassData passData))
            {
                passData.settings = settings;
                passData.voxelCompute = voxelCompute;
                passData.voxel0Handle = voxel0Handle;
                builder.AllowPassCulling(false);
                builder.UseTexture(voxel0Handle, AccessFlags.Write);
                builder.SetRenderFunc<PassData>(ClearFunc);
            }

            using (var builder = renderGraph.AddRasterRenderPass("Voxel Cascade 0 Render", out PassData passData))
            {
                passData.rendererList = rendererList0;
                passData.settings = settings;
                passData.cameraCenter = cameraData.worldSpaceCameraPos;
                passData.voxel0Handle = voxel0Handle;
                builder.AllowPassCulling(false);
                builder.SetGlobalTextureAfterPass(voxel0Handle, ShaderIds._VoxelCascade0);
                builder.UseRendererList(rendererList0);
                builder.SetRenderAttachmentDepth(voxelTargetDepthHandle);
                builder.SetRenderAttachment(voxelTargetColorHandle, 0);
                builder.SetRandomAccessAttachment(voxel0Handle, 1);
                builder.AllowGlobalStateModification(true);
                builder.SetRenderFunc<PassData>(VoxelizeFunc);
            }
        }

        static void ClearFunc (PassData passData, ComputeGraphContext context)
        {
            var cmd = context.cmd;
            var voxelCompute = passData.voxelCompute;
            int clearKernel = passData.voxelKernel;
            int resolution = passData.settings.voxelCascadeResolution;
            int clearThreadGroup = Mathf.CeilToInt(resolution / 4f);

            cmd.SetComputeTextureParam(voxelCompute, clearKernel, ShaderIds._VoxelCascade0, passData.voxel0Handle);
            cmd.SetComputeIntParam(voxelCompute, ShaderIds.resolution, resolution);
            cmd.DispatchCompute(voxelCompute, clearKernel, clearThreadGroup, clearThreadGroup, clearThreadGroup);
        }

        static Matrix4x4[] viewProjectMats = new Matrix4x4[3];
        static void VoxelizeFunc (PassData passData, RasterGraphContext context)
        {
            var cmd = context.cmd;
            int resolution = passData.settings.voxelCascadeResolution;
            float scale = passData.settings.voxelBaseCascadeScale;

            float extent = resolution*scale/2f;
            Vector3 center = math.round(passData.cameraCenter / scale) * scale;
            Vector3 volumeCenter = center - new Vector3(extent, extent, extent);

            var camOrthoNORMAL = Matrix4x4.Ortho(-extent, extent, -extent, extent, 0, extent * 2);
            var camOrtho = GL.GetGPUProjectionMatrix(Matrix4x4.Ortho(-extent, extent, -extent, extent, 0, extent * 2), renderIntoTexture: true);
            var camTRS0 = Matrix4x4.TRS(new Vector3(center.x + extent, center.y, center.z), Quaternion.Euler(0, -90, 0), new Vector3(1, 1, -1));
            var camTRS1 = Matrix4x4.TRS(new Vector3(center.x, center.y + extent, center.z), Quaternion.Euler(90, 0, 0), new Vector3(1, 1, -1));
            var camTRS2 = Matrix4x4.TRS(new Vector3(center.x, center.y, center.z + extent), Quaternion.Euler(0, 180, 0), new Vector3(1, 1, -1));
            viewProjectMats[0] = camOrtho * camTRS0.inverse;
            viewProjectMats[1] = camOrtho * camTRS1.inverse;
            viewProjectMats[2] = camOrtho * camTRS2.inverse;

            // Setup shared values
            cmd.SetGlobalMatrixArray(ShaderIds.ViewProjectionMatrix, viewProjectMats);
            cmd.SetGlobalTexture(ShaderIds._VoxelCascade0, passData.voxel0Handle);
            cmd.SetGlobalInt(ShaderIds.resolution, resolution);
            cmd.SetGlobalFloat(ShaderIds.invScale, 1f / scale);
            cmd.SetGlobalVector(ShaderIds.VolumeCenter, new Vector4(volumeCenter.x, volumeCenter.y, volumeCenter.z, 0));

            //RenderingUtils.SetViewAndProjectionMatrices(cmd, camTRS0, camOrthoNORMAL, setInverseMatrices: true);
            cmd.SetViewport(new Rect(0, 0, resolution, resolution));
            cmd.DrawRendererList(passData.rendererList);

        }
    }
}