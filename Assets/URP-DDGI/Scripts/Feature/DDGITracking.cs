using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace DDGIURP
{
    /// <summary>
    /// Section of the DDGI feature that takes care of tracking all visible renderers
    /// This will enable us to do per-cascade culling to speed up the voxel rendering.
    /// </summary>
    public partial class DDGIFeature : ScriptableRendererFeature
    {
        // Renderer tracking (Voxel Only)
        public override void OnCameraPreCull (ScriptableRenderer renderer, in CameraData cameraData)
        {
            //var cullParams = new ScriptableCullingParameters();
            //var camData = new CameraData();
            //renderer.SetupCullingParameters(ref cullParams, ref camData);
        }

        private void BeginContextRendering (ScriptableRenderContext context, List<Camera> cameras)
        {
            /*var cullingResults = new CullingResults();
            var drawingSettings = new DrawingSettings();
            var sortingSettings = new SortingSettings();
            sortingSettings.criteria = SortingCriteria.None;
            drawingSettings.sortingSettings = sortingSettings;
            var filteringSettings = new FilteringSettings();
            var renderListParams = new RendererListParams(cullingResults, drawingSettings, filteringSettings);
            var renderList = context.CreateRendererList(ref renderListParams);


            foreach (Camera camera in cameras)
            {
                Debug.Log(camera);
            }*/
        }
    }
}