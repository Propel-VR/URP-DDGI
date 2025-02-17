using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace DDGIURP
{
    public partial class DDGIFeature : ScriptableRendererFeature
    {
        [SerializeField] DDGISettings settings;

        
        static DDGIFeature inst;
        public static int VoxelLayer => inst == null ? 31 : inst.settings.voxelRendererLayer;

        public override void AddRenderPasses (ScriptableRenderer renderer, ref RenderingData renderingData)
        {

        }

        public override void Create ()
        {
            inst = this;

            RenderPipelineManager.beginContextRendering += BeginContextRendering;
        }

        protected override void Dispose (bool disposing)
        {
            RenderPipelineManager.beginContextRendering -= BeginContextRendering;
        }
    }
}