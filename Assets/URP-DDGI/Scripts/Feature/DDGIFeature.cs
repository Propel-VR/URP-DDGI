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
        [SerializeField] RenderTexture debugTexture2;

        // Resources
        VoxelizePass voxelPass;
        VoxelDebugPass voxelDebugPass;
        Camera dummyVoxelCamera0;
        Camera primaryVoxelCamera;

        static bool eventAssigned;
        static DDGIFeature inst;
        public static int VoxelLayer => inst == null ? 31 : inst.settings.voxelRendererLayer;

        public override void AddRenderPasses (ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (renderingData.cameraData.camera != primaryVoxelCamera) return;

            renderer.EnqueuePass(voxelPass);

            debugTexture = voxelPass.voxel0Volume.rt;
            debugTexture2 = voxelPass.voxelTargetColor.rt;

            if (DDGIDebugPanel.VoxelEnabled)
            {
                renderer.EnqueuePass(voxelDebugPass);
            }
        }

        public override void Create ()
        {
            inst = this;

            FreeResources();
            AllocateResources();
        }

        int lastFrame = 0;
        private void BeginContextRendering (ScriptableRenderContext context, List<Camera> cameras)
        {

            // The camera the voxel will be centered around.
            primaryVoxelCamera = cameras[0];

            // Ensure there is a fake camera. We need it for culling stuff?
            if (voxelPass == null) return;
            if (dummyVoxelCamera0 == null)
            {
                var dummyVoxelCamObject = new GameObject("[DDGI] Fake Voxel Camera");
                dummyVoxelCamObject.hideFlags = HideFlags.DontSaveInBuild | HideFlags.DontSaveInEditor;
                dummyVoxelCamera0 = dummyVoxelCamObject.AddComponent<Camera>();
            }
            float extent = settings.voxelBaseCascadeScale * settings.voxelCascadeResolution / 2f;
            dummyVoxelCamera0.enabled = false;
            dummyVoxelCamera0.orthographic = true;
            dummyVoxelCamera0.aspect = 1;
            dummyVoxelCamera0.orthographicSize = extent;
            dummyVoxelCamera0.nearClipPlane = 0;
            dummyVoxelCamera0.farClipPlane = extent * 2;
            dummyVoxelCamera0.cullingMask = 1 << settings.voxelRendererLayer;
            dummyVoxelCamera0.useOcclusionCulling = false;
            dummyVoxelCamera0.clearFlags = CameraClearFlags.SolidColor;
            dummyVoxelCamera0.backgroundColor = Color.black;
            dummyVoxelCamera0.transform.position = primaryVoxelCamera.transform.position + Vector3.back * extent;
            voxelPass.SetCamera(dummyVoxelCamera0);
        }

        private void AllocateResources ()
        {
            voxelPass = new VoxelizePass(settings);
            voxelDebugPass = new VoxelDebugPass(settings);

            if (!eventAssigned)
            {
                eventAssigned = true;
                RenderPipelineManager.beginContextRendering += BeginContextRendering;
            }
        }

        private void FreeResources ()
        {
            if (voxelPass != null)
            {
                voxelPass.Dispose();
                voxelPass = null;
            }
            if (voxelDebugPass != null)
            {
                voxelDebugPass.Dispose();
            }
        }

        protected override void Dispose (bool disposing)
        {
            FreeResources();
            if(eventAssigned)
            {
                RenderPipelineManager.beginContextRendering -= BeginContextRendering;
                eventAssigned = false;
            }
            if (dummyVoxelCamera0 != null)
            {
                DestroyImmediate(dummyVoxelCamera0.gameObject);
            }
        }

        private void OnEnable ()
        {
            
            DDGIDebugPanel.CreatePanel();
        }

        private void OnDisable ()
        {
            
            DDGIDebugPanel.RemovePanel();
        }
    }
}