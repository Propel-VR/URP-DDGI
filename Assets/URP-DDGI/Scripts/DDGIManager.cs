using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using Unity.Collections;


#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DDGIURP 
{

    [ExecuteInEditMode, RequireComponent(typeof(DDGIDebugProbeRenderer))]
    public class DDGIManager : MonoBehaviour
    {
        [Header("Probe Settings")]
        [SerializeField] Vector3 boxCenter;
        [SerializeField] Vector3 boxSize;
        [SerializeField] float probeDensity;

        [Header("Rendering Settings")]
        [SerializeField, Range(4, 48)] int radianceResolution = 6;
        [SerializeField, Range(4, 48)] int visibilityResolution = 12;

        [Header("Debug Settings")]
        [SerializeField] int maxMemoryUsageMB = 1000;
        [SerializeField] bool enabledProbeDebuggingView;

        [ReadOnly, SerializeField] int3 probeDimensions;
        [ReadOnly, SerializeField] int probeCount;
        [ReadOnly, SerializeField] float memoryUsageInMB;

        RenderTexture radianceTex;
        RenderTexture visibilityTex;
        DDGIDebugProbeRenderer debugProbeRenderer;

        static readonly int[] validResolutions = { 4, 6, 8, 12, 16, 24, 48 };
        const int bytesPerRadiancePixel = 4; // If using R11G11B10
        const int bytesPerDepthPixel = 2; // If using F16

        static DDGIManager inst;

        public RenderTexture RadianceTexture => radianceTex;
        public RenderTexture VisibilityTexture => visibilityTex;


        private void Awake()
        {
            inst = this;
            RefreshProbes();
        }

        private void OnValidate() => RefreshProbes();

        private void OnDestroy() => Cleanup();

        private void RefreshProbes ()
        {
            debugProbeRenderer = GetComponent<DDGIDebugProbeRenderer>();

            probeDimensions = (int3)math.round(boxSize / probeDensity);
            probeCount = probeDimensions.x * probeDimensions.y * probeDimensions.z;

            radianceResolution = validResolutions.OrderBy(res => math.abs(res - radianceResolution)).FirstOrDefault();
            visibilityResolution = validResolutions.OrderBy(res => math.abs(res - visibilityResolution)).FirstOrDefault();

            int radResPad = radianceResolution + 2;
            int visResPad = visibilityResolution + 2;

            int bytesPerProbe = radResPad * radResPad * bytesPerRadiancePixel +
                visResPad * visResPad * bytesPerDepthPixel;

            memoryUsageInMB = bytesPerProbe * probeCount / 1000_000f;

            debugProbeRenderer.UpdateLayout(probeDimensions, probeDensity, boxCenter, boxSize);

            Cleanup();
            Allocate();
        }

        public void Cleanup ()
        {
            if(radianceTex)
            {
                DestroyImmediate(radianceTex);
            }
            if(visibilityTex)
            {
                DestroyImmediate(visibilityTex);
            }
        }

        public void Allocate ()
        {
            if (memoryUsageInMB > maxMemoryUsageMB) return;


            int radResPad = radianceResolution + 2;
            int visResPad = visibilityResolution + 2;

            int radianceTexWidth = probeDimensions.x * radResPad;
            int radianceTexHeight = probeDimensions.y * radResPad;
            int radianceTexDepth = probeDimensions.z * radResPad;

            radianceTex = new RenderTexture(
                width: radianceTexWidth,
                height: radianceTexHeight,
                depth: 0,
                format: RenderTextureFormat.RGB111110Float,
                mipCount: 0
            ){
                enableRandomWrite = true,
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = radianceTexDepth
            };

            int visibilityTexWidth = probeDimensions.x * visResPad;
            int visibilityTexHeight = probeDimensions.y * visResPad;
            int visibilityTexDepth = probeDimensions.z * visResPad;

            visibilityTex = new RenderTexture(
                width: visibilityTexWidth,
                height: visibilityTexHeight,
                depth: 0,
                format: GraphicsFormat.R16G16_SFloat,
                mipCount: 0
            )
            {
                enableRandomWrite = true,
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = visibilityTexDepth
            };
        }


        private void OnDrawGizmosSelected ()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(boxCenter + transform.position, boxSize);
        }
    }
}
