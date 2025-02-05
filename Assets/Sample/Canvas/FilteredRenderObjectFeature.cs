
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;
using static UnityEngine.Rendering.Universal.RenderObjects;

public class FilteredRenderObjectFeature : ScriptableRendererFeature
{
    [System.Serializable]
    public class Settings
    {
        public LayerMask layerMask = -1;
    }

    class FilteredGeometryPass : ScriptableRenderPass
    {
        // Across all cameras
        string name;
        static public List<ShaderTagId> shaderTagIds = new()
        {
            new ShaderTagId("UniversalForwardOnly"),
            new ShaderTagId("UniversalForward"),
            new ShaderTagId("SRPDefaultUnlit"),
        };
        public FilteringSettings filterSettings;
        public RenderQueueType renderQueueType;
        public bool isUI;


        // Specific to a camera
        class PassData
        {
            public RendererListHandle rendererListHandle;
        }

        public FilteredGeometryPass (string name, RenderQueueType renderQueueType)
        {
            this.name = name;
            this.renderQueueType = renderQueueType;
        }


        void InitRendererList(ContextContainer frameData, ref PassData passData, RenderGraph renderGraph)
        {
            // Access the relevant frame data from the Universal Render Pipeline
            var universalRenderingData = frameData.Get<UniversalRenderingData>();
            var cameraData = frameData.Get<UniversalCameraData>();
            var lightData = frameData.Get<UniversalLightData>();
            var sortFlag = (renderQueueType == RenderQueueType.Transparent)
                ? SortingCriteria.CommonTransparent
                : cameraData.defaultOpaqueSortFlags;

            var camera = cameraData.camera;
            camera.TryGetComponent<CameraRenderingLayer>(out var camLayerComponent);
            var renderingLayerMask = (camLayerComponent != null) ? camLayerComponent.Mask : (RenderingLayerMask)uint.MaxValue;
            var uiChannel = (camLayerComponent != null) ? camLayerComponent.CanvasChannel : (short)0;

            if (isUI)
            {
                renderingLayerMask = (RenderingLayerMask)uint.MaxValue;
                filterSettings.sortingLayerRange = new SortingLayerRange(uiChannel, uiChannel);
            }

            if(cameraData.isSceneViewCamera)
            {
                renderingLayerMask = (RenderingLayerMask)uint.MaxValue;
                filterSettings.sortingLayerRange = SortingLayerRange.all;
            }

            filterSettings.renderingLayerMask = renderingLayerMask;

            var drawSettings = RenderingUtils.CreateDrawingSettings(shaderTagIds, universalRenderingData, cameraData, lightData, sortFlag);
            var param = new RendererListParams(universalRenderingData.cullResults, drawSettings, filterSettings);
            passData.rendererListHandle = renderGraph.CreateRendererList(param);
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            using var builder = renderGraph.AddRasterRenderPass<PassData>(name, out var passData);

            var resourceData = frameData.Get<UniversalResourceData>();

            // We declare the RendererList we just created as an input dependency to this pass, via UseRendererList()
            InitRendererList(frameData, ref passData, renderGraph);
            builder.UseRendererList(passData.rendererListHandle);

            builder.SetRenderAttachment(resourceData.activeColorTexture, 0);

            // Assign the ExecutePass function to the render pass delegate, which will be called by the render graph when executing the pass
            builder.SetRenderFunc((PassData data, RasterGraphContext context) => ExecutePass(data, context));
            builder.SetRenderFunc<PassData>(ExecutePass);

            builder.AllowPassCulling(false);
            builder.UseAllGlobalTextures(true);
        }

        static void ExecutePass(PassData data, RasterGraphContext context)
        {
            context.cmd.DrawRendererList(data.rendererListHandle);
        }
    }



    [SerializeField] Settings m_opaquePassSettings = new();
    [SerializeField] Settings m_transparentPassSettings = new();
    [SerializeField] Settings m_uiPassSettings = new();

    private FilteredGeometryPass m_opaquePass; 
    private FilteredGeometryPass m_transparentPass; 
    private FilteredGeometryPass m_uiPass; 
    static Dictionary<Camera, CameraRenderingLayer> m_cameraRenderingLayerCache = new();

    public override void Create()
    {
        m_opaquePass = new FilteredGeometryPass("Opaque Filtered Pass", RenderQueueType.Opaque)
        {
            renderPassEvent = RenderPassEvent.AfterRenderingOpaques,
            filterSettings = new FilteringSettings(
                renderQueueRange: RenderQueueRange.opaque, 
                layerMask: m_opaquePassSettings.layerMask)
        };

        m_transparentPass = new FilteredGeometryPass("Transparent Filtered Pass", RenderQueueType.Transparent)
        {
            renderPassEvent = RenderPassEvent.AfterRenderingTransparents,
            filterSettings = new FilteringSettings(
                renderQueueRange: RenderQueueRange.transparent,
                layerMask: m_transparentPassSettings.layerMask)
        };

        m_uiPass = new FilteredGeometryPass("UI Filtered Pass", RenderQueueType.Transparent)
        {
            renderPassEvent = RenderPassEvent.AfterRenderingTransparents + 1,
            filterSettings = new FilteringSettings(
                renderQueueRange: RenderQueueRange.transparent,
                layerMask: m_uiPassSettings.layerMask),
            isUI = true
        };
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        m_opaquePass.filterSettings.layerMask = m_opaquePassSettings.layerMask;
        m_transparentPass.filterSettings.layerMask = m_transparentPassSettings.layerMask;
        m_uiPass.filterSettings.layerMask = m_uiPassSettings.layerMask;

        renderer.EnqueuePass(m_opaquePass);
        renderer.EnqueuePass(m_transparentPass);
        renderer.EnqueuePass(m_uiPass);
    }
}