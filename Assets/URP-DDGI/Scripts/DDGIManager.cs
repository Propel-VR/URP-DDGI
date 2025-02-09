using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using Unity.Collections;
using EasyButtons;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DDGIURP 
{
    [ExecuteInEditMode, RequireComponent(typeof(DDGIDebugProbeRenderer), typeof(BVHScene))]
    public class DDGIManager : MonoBehaviour
    {
        [Header("Probe Settings")]
        [SerializeField] Vector3 boxCenter;
        [SerializeField] Vector3 boxSize;
        [SerializeField] float probeDensity;

        [Header("Rendering Settings")]
        [SerializeField, Range(4, 48)] int irradianceResolution = 6;
        [SerializeField, Range(4, 48)] int visibilityResolution = 12;

        [Header("Debug Settings")]
        [SerializeField] int maxMemoryUsageMB = 1000;
        [SerializeField] bool enabledProbeDebuggingView;

        [ReadOnly, SerializeField] int3 probeDimensions;
        [ReadOnly, SerializeField] int probeCount;
        [ReadOnly, SerializeField] float memoryUsageInMB;
        [ReadOnly, SerializeField] RenderTexture irradianceTex;
        [ReadOnly, SerializeField] RenderTexture visibilityTex;
        [ReadOnly, SerializeField] RenderTexture tracedTargetTex;

        ComputeShader DDGI;
        DDGIDebugProbeRenderer debugProbeRenderer;
        BVHScene scene;
        int irrResPad;
        int visResPad;

        static readonly int[] validResolutions = { 4, 6, 8, 12, 16, 24, 48 };
        const int bytesPerIrradiancePixel = 4; // If using R11G11B10
        const int bytesPerDepthPixel = 4; // If using R16G16z

        static DDGIManager inst;

        public RenderTexture IrradianceTexture => irradianceTex;
        public RenderTexture VisibilityTexture => visibilityTex;


        private void Awake()
        {
            inst = this;
            RefreshProbes();

            DDGI = Resources.Load<ComputeShader>("Shaders/DDGI");
        }

        private void OnValidate() => RefreshProbes();

        void OnDisable() => Cleanup();

        private void RefreshProbes ()
        {
            scene = GetComponent<BVHScene>();

            debugProbeRenderer = GetComponent<DDGIDebugProbeRenderer>();
            debugProbeRenderer.enabled = enabledProbeDebuggingView;

            boxSize = Vector3.Max(boxSize, Vector3.zero);

            probeDimensions = (int3)math.round(boxSize / probeDensity);
            probeCount = probeDimensions.x * probeDimensions.y * probeDimensions.z;

            irradianceResolution = validResolutions.OrderBy(res => math.abs(res - irradianceResolution)).FirstOrDefault();
            visibilityResolution = validResolutions.OrderBy(res => math.abs(res - visibilityResolution)).FirstOrDefault();

            irrResPad = irradianceResolution + 2;
            visResPad = visibilityResolution + 2;

            int bytesPerProbe = irrResPad * irrResPad * bytesPerIrradiancePixel +
                visResPad * visResPad * bytesPerDepthPixel;

            memoryUsageInMB = bytesPerProbe * probeCount / 1000_000f;


            Cleanup();
            Allocate();

            debugProbeRenderer.UpdateLayout(this, probeDimensions, probeDensity, boxCenter, boxSize, irrResPad, irradianceResolution);
        }

        public void Allocate ()
        {
            if (memoryUsageInMB > maxMemoryUsageMB) return;


            int irrResPad = irradianceResolution + 2;
            int visResPad = visibilityResolution + 2;

            int irradianceTexWidth = probeDimensions.x * irrResPad;
            int irradianceTexHeight = probeDimensions.y * irrResPad;
            int irradianceTexDepth = probeDimensions.z;

            irradianceTex = new RenderTexture(
                width: irradianceTexWidth,
                height: irradianceTexHeight,
                depth: 0,
                format: RenderTextureFormat.RGB111110Float,
                mipCount: 0
            ){
                name = "DDGI Irradiance",
                enableRandomWrite = true,
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = irradianceTexDepth,
                anisoLevel = 0,
                filterMode = FilterMode.Bilinear,
            };
            irradianceTex.Create();

            int visibilityTexWidth = probeDimensions.x * visResPad;
            int visibilityTexHeight = probeDimensions.y * visResPad;
            int visibilityTexDepth = probeDimensions.z;

            visibilityTex = new RenderTexture(
                width: visibilityTexWidth,
                height: visibilityTexHeight,
                depth: 0,
                format: GraphicsFormat.R16G16_SFloat,
                mipCount: 0
            )
            {
                name = "DDGI Visbility",
                enableRandomWrite = true,
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = visibilityTexDepth,
                anisoLevel = 0,
                filterMode = FilterMode.Bilinear,
            };
            visibilityTex.Create();

            tracedTargetTex = new RenderTexture(
                width: 4096,
                height: 4096,
                depth: 0,
                format: GraphicsFormat.R16G16B16A16_SFloat,
                mipCount: 0
            )
            {
                name = "DDGI Traced Results",
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = 1,
            };

            Debug.Log("Recreated Textures");
        }

        public void Cleanup()
        {
            if (irradianceTex) DestroyImmediate(irradianceTex);
            if (visibilityTex) DestroyImmediate(visibilityTex);
            if (tracedTargetTex) DestroyImmediate(tracedTargetTex);
        }

        [Button]
        public void FillWithDebugData ()
        {
            var cmd = CommandBufferPool.Get("DDGI Fill Debug Data");

            int kernel = DDGI.FindKernel("FillIrradianceWithDebugData");
            int threadCountX = (int)math.ceil(probeDimensions.x * irrResPad / 8f);
            int threadCountY = (int)math.ceil(probeDimensions.y * irrResPad / 8f);
            int threadCountZ = probeDimensions.z;

            cmd.SetComputeIntParam(DDGI, ShaderIds.irrRes, irradianceResolution);
            cmd.SetComputeIntParam(DDGI, ShaderIds.visRes, visibilityResolution);
            cmd.SetComputeIntParam(DDGI, ShaderIds.irrResPad, irrResPad);
            cmd.SetComputeIntParam(DDGI, ShaderIds.visResPad, visResPad);
            cmd.SetComputeIntParams(DDGI, ShaderIds.probeDimensions, PackInt3(probeDimensions));
            cmd.SetComputeTextureParam(DDGI, kernel, ShaderIds._Visibility, visibilityTex);
            cmd.SetComputeTextureParam(DDGI, kernel, ShaderIds._Irradiance, irradianceTex);
            cmd.DispatchCompute(DDGI, kernel, threadCountX, threadCountY, threadCountZ);

            Graphics.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
            Debug.Log("Executed Command");
        }


        private void OnDrawGizmosSelected ()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(boxCenter + transform.position, boxSize);
        }


        // Utilities to avoids allocations
        static int[] _tempInt3 = new int[3];
        static int[] PackInt3 (int3 value)
        {
            _tempInt3[0] = value[0];
            _tempInt3[1] = value[1];
            _tempInt3[2] = value[2];
            return _tempInt3;
        }
    }
}
