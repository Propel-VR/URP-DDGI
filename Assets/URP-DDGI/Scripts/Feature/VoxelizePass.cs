using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering.RenderGraphModule.Util;

namespace DDGIURP
{
    public class VoxelizePass : ScriptableRenderPass, System.IDisposable
    {
        internal DDGISettings settings;
        internal RTHandle voxel0Volume;
        internal CullingResults voxel0CullResults;

        private TextureDesc voxelTargetDesc;
        private TextureHandle voxelTarget;
        private RendererListHandle rendererList0;
        private ComputeShader voxelCompute;
        private int voxelKernel;
        private Shader voxelShader;
        private Material overrideVoxelMat;

        internal class PassData
        {
            public DDGISettings settings;
            public RendererListHandle rendererList;
            public ComputeShader voxelCompute;
            public int voxelKernel;
            public TextureHandle voxel0Handle;
            public Vector3 cameraCenter;
            public Material overrideVoxelMat;
        }

        public VoxelizePass (DDGISettings settings)
        {
            this.renderPassEvent = RenderPassEvent.AfterRenderingShadows;

            this.settings = settings;
            FreeResources();
            AllocateResources();
        }

        public void Dispose ()
        {
            FreeResources();
        }

        private void AllocateResources ()
        {
            voxelTargetDesc = new TextureDesc()
            {
                name = "VoxelRenderTarget",
                dimension = TextureDimension.Tex2D,
                width = settings.voxelCascadeResolution,
                height = settings.voxelCascadeResolution,
                slices = 1,
                format = GraphicsFormat.R8_UNorm,
                depthBufferBits = DepthBits.None,
                useMipMap = false,
                msaaSamples = MSAASamples.None,
            };

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

            voxelCompute = Resources.Load<ComputeShader>("Shaders/Voxel");
            voxelKernel = voxelCompute.FindKernel("Clear");
            voxelShader = Resources.Load<Shader>("Shaders/VoxelShader2");
            overrideVoxelMat = new Material(voxelShader);
        }

        void FreeResources ()
        {
            if (voxel0Volume != null)
            {
                RTHandles.Release(voxel0Volume);
                voxel0Volume = null;
            }
            if(overrideVoxelMat != null)
            {
                Object.DestroyImmediate(overrideVoxelMat);
                overrideVoxelMat = null;
            }
        }

        void CreateRenderLists (RenderGraph renderGraph, UniversalCameraData cameraData)
        {
            var sortingSettings = new SortingSettings()
            {
                cameraPosition = cameraData.worldSpaceCameraPos,
                criteria = SortingCriteria.None
            };
            var drawSettings = new DrawingSettings()
            {
                overrideMaterial = overrideVoxelMat,
                overrideMaterialPassIndex = 0,
                sortingSettings = sortingSettings,
                enableDynamicBatching = true,
                enableInstancing = true,
                perObjectData = PerObjectData.None
            };
            var filteringSettings = new FilteringSettings()
            {
                layerMask = 1 << settings.voxelRendererLayer
            };
            var renderListParams0 = new RendererListParams(voxel0CullResults, drawSettings, filteringSettings);
            rendererList0 = renderGraph.CreateRendererList(renderListParams0);
        }

        public override void RecordRenderGraph (RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();
            var cameraData = frameData.Get<UniversalCameraData>();

            CreateRenderLists(renderGraph, cameraData);

            var voxel0Handle = renderGraph.ImportTexture(voxel0Volume);
            var voxelTargetHandle = renderGraph.CreateTexture(voxelTargetDesc);

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
                passData.overrideVoxelMat = overrideVoxelMat;
                builder.AllowPassCulling(false);
                builder.SetGlobalTextureAfterPass(voxel0Handle, ShaderIds._VoxelCascade0);
                builder.UseRendererList(rendererList0);
                builder.SetRenderAttachment(voxelTargetHandle, 0, AccessFlags.Write);
                builder.SetRandomAccessAttachment(voxel0Handle, 1);
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

            cmd.SetComputeTextureParam(voxelCompute, clearKernel, ShaderIds._VoxelTarget, passData.voxel0Handle);
            cmd.SetComputeIntParam(voxelCompute, ShaderIds.resolution, resolution);
            cmd.DispatchCompute(voxelCompute, clearKernel, clearThreadGroup, clearThreadGroup, clearThreadGroup);
        }

        static Matrix4x4[] viewProjectMats = new Matrix4x4[3];
        static void VoxelizeFunc (PassData passData, RasterGraphContext context)
        {
            var cmd = context.cmd;
            int resolution = passData.settings.voxelCascadeResolution;
            float scale = passData.settings.voxelBaseCascadeScale;
            var overrideVoxelMat = passData.overrideVoxelMat;

            float extent = resolution*scale/2f;
            Vector3 center = passData.cameraCenter;
            Vector3 volumeCenter = center - new Vector3(extent, extent, extent);

            var camOrtho = GL.GetGPUProjectionMatrix(Matrix4x4.Ortho(-extent, extent, -extent, extent, 0, extent * 2), renderIntoTexture: true);
            var camTRS0 = Matrix4x4.TRS(new Vector3(center.x + extent, center.y, center.z), Quaternion.Euler(0, -90, 0), new Vector3(1, 1, -1));
            var camTRS1 = Matrix4x4.TRS(new Vector3(center.x, center.y + extent, center.z), Quaternion.Euler(90, 0, 0), new Vector3(1, 1, -1));
            var camTRS2 = Matrix4x4.TRS(new Vector3(center.x, center.y, center.z + extent), Quaternion.Euler(0, 180, 0), new Vector3(1, 1, -1));
            viewProjectMats[0] = camOrtho * camTRS0.inverse;
            viewProjectMats[1] = camOrtho * camTRS1.inverse;
            viewProjectMats[2] = camOrtho * camTRS2.inverse;

            // Setup shared values
            overrideVoxelMat.SetMatrixArray(ShaderIds.ViewProjectionMatrix, viewProjectMats);
            overrideVoxelMat.SetTexture(ShaderIds._VoxelTarget, passData.voxel0Handle);
            overrideVoxelMat.SetInt(ShaderIds.resolution, resolution);
            overrideVoxelMat.SetFloat(ShaderIds.invScale, 1f / scale);
            overrideVoxelMat.SetVector(ShaderIds.VolumeCenter, new Vector4(volumeCenter.x, volumeCenter.y, volumeCenter.z, 0));

            cmd.SetViewport(new Rect(0, 0, resolution, resolution));
            cmd.DrawRendererList(passData.rendererList);
        }
    }
}