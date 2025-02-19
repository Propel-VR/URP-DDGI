using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace DDGIURP
{
    public partial class DDGIFeature : ScriptableRendererFeature
    {
        [SerializeField] DDGISettings settings;

        [SerializeField] RenderTexture debugTexture;

        // Resources
        VoxelizePass voxelPass;
        Camera dummyVoxelCamera0;
        Camera primaryVoxelCamera;

        static DDGIFeature inst;
        public static int VoxelLayer => inst == null ? 31 : inst.settings.voxelRendererLayer;

        public override void AddRenderPasses (ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (renderingData.cameraData.camera != primaryVoxelCamera) return;

            renderer.EnqueuePass(voxelPass);

            debugTexture = voxelPass.voxel0Volume.rt;
        }

        public override void Create ()
        {
            inst = this;

            FreeResources();
            voxelPass = new VoxelizePass(settings);
        }

        int lastFrame = 0;
        private void BeginContextRendering (ScriptableRenderContext context, List<Camera> cameras)
        {
            // This is to avoid game view calling the pipeline twice.
            // Our DDGI system should only ever run once per frame
            if(lastFrame == Time.frameCount)
            {
                return;
            }
            lastFrame = Time.frameCount;

            // The camera the voxel will be centered around.
            primaryVoxelCamera = cameras[0];

            // Ensure there is a fake camera. We need it for culling stuff.
            if (voxelPass == null) return;
            if (dummyVoxelCamera0 == null)
            {
                var dummyVoxelCamObject = new GameObject("[DDGI] Fake Voxel Camera");
                dummyVoxelCamera0 = dummyVoxelCamObject.AddComponent<Camera>();
                
                dummyVoxelCamera0.enabled = false;
                dummyVoxelCamera0.orthographic = true;
                dummyVoxelCamera0.aspect = 1;
            }
            float extent = settings.voxelBaseCascadeScale * settings.voxelCascadeResolution / 2f;
            dummyVoxelCamera0.orthographicSize = extent;
            dummyVoxelCamera0.nearClipPlane = 0;
            dummyVoxelCamera0.farClipPlane = extent * 2;
            dummyVoxelCamera0.transform.position = primaryVoxelCamera.transform.position + Vector3.back * extent;
            dummyVoxelCamera0.cullingMask = (LayerMask)settings.voxelRendererLayer;

            if(dummyVoxelCamera0.TryGetCullingParameters(out var voxel0CullParams)) 
            {
                voxelPass.voxel0CullResults = context.Cull(ref voxel0CullParams);
            }
            else
            {
                voxelPass.voxel0CullResults = default;
            }
        }

        private void FreeResources ()
        {
            if (voxelPass != null)
            {
                voxelPass = null;
            }
        }

        protected override void Dispose (bool disposing)
        {
            FreeResources();
        }

        private void OnEnable ()
        {
            RenderPipelineManager.beginContextRendering += BeginContextRendering;
            DDGIDebugPanel.CreatePanel();
        }

        private void OnDisable ()
        {
            RenderPipelineManager.beginContextRendering -= BeginContextRendering;
            DDGIDebugPanel.RemovePanel();

            if(dummyVoxelCamera0 != null)
            {
                DestroyImmediate(dummyVoxelCamera0.gameObject);
            }
        }
    }
}