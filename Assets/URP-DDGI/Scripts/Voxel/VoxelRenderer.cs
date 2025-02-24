using UnityEngine;
using UnityEngine.Rendering;
using static UnityEngine.Rendering.Universal.UniversalRenderPipeline;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DDGIURP
{

    /// <summary>
    /// This components enabled the tracking of mesh renderers for the purpose of batched 
    /// voxel-rendering, while not requiring Geometry shaders.
    /// 
    /// It's secret is the generation of tris-split meshes within burst jobs.
    /// </summary>
    [RequireComponent(typeof(Renderer)), DefaultExecutionOrder(10)]
    public class VoxelRenderer : MonoBehaviour
    {
        private Renderer sourceRenderer;
        private Renderer renderer;
        private Transform sourceTransform;
        private GameObject rendererCopy;
        private Transform copyTransform;
        private Mesh registeredMesh;

        private Material[] replacementMaterials;

        public void Refresh ()
        {

        }

        internal void OnUpdateTransform ()
        {
            copyTransform.parent = transform.parent;
            copyTransform.SetLocalPositionAndRotation(transform.localPosition, transform.localRotation);
            copyTransform.localScale = transform.localScale;
        }

        private void Start ()
        {
            sourceRenderer = GetComponent<Renderer>();
            sourceTransform = transform;

            if(!(sourceRenderer is MeshRenderer || sourceRenderer is SkinnedMeshRenderer))
            {
                return;
            }

            VoxelRendererManager.AddRenderer(this, gameObject.isStatic);

            rendererCopy = new GameObject(gameObject.name + " (VoxelRenderer)");
            //rendererCopy.hideFlags = HideFlags.HideAndDontSave;
            var copyTransform = rendererCopy.transform;
            copyTransform.parent = transform.parent;
            copyTransform.SetLocalPositionAndRotation(transform.localPosition, transform.localRotation);
            copyTransform.localScale = transform.localScale;
            rendererCopy.layer = DDGIFeature.VoxelLayer;


            if(sourceRenderer is MeshRenderer)
            {
                var meshFilter = rendererCopy.AddComponent<MeshFilter>();
                var sourceMeshFilter = GetComponent<MeshFilter>();

                registeredMesh = sourceMeshFilter.sharedMesh;
                var splitMesh = VoxelRendererManager.AddMesh(registeredMesh, isSkinned: false);
                meshFilter.sharedMesh = splitMesh;

                var meshRenderer = rendererCopy.AddComponent<MeshRenderer>();
                var sourceMeshRenderer = GetComponent<MeshRenderer>();
                renderer = meshRenderer;
            }
            else if(sourceRenderer is SkinnedMeshRenderer)
            {
                var skinnedRenderer = rendererCopy.AddComponent<SkinnedMeshRenderer>();
                var sourceSkinnedRenderer = GetComponent<SkinnedMeshRenderer>();

                registeredMesh = skinnedRenderer.sharedMesh;
                var splitMesh = VoxelRendererManager.AddMesh(registeredMesh, isSkinned: true);
                skinnedRenderer.sharedMesh = splitMesh;
                skinnedRenderer.rootBone = sourceSkinnedRenderer.rootBone;
                renderer = skinnedRenderer;
            }

            renderer.shadowCastingMode = ShadowCastingMode.Off;
            renderer.rayTracingMode = UnityEngine.Experimental.Rendering.RayTracingMode.Off;
            renderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
            renderer.receiveShadows = false;

            var voxelShader = Resources.Load<Shader>("Shaders/VoxelShader");
            var sourceMats = sourceRenderer.sharedMaterials;
            replacementMaterials = new Material[sourceMats.Length];
            for(int i = 0; i < sourceMats.Length; i++)
            {
                replacementMaterials[i] = new Material(sourceMats[i]);
                replacementMaterials[i].shader = voxelShader;
            }
            renderer.sharedMaterials = replacementMaterials;
        }

        private void OnDestroy ()
        {
            if (rendererCopy == null) return;

            VoxelRendererManager.RemoveRenderer(this);

            if (registeredMesh != null)
            {
                VoxelRendererManager.RemoveMesh(registeredMesh);
                registeredMesh = null;
            }

#if UNITY_EDITOR
            if (EditorApplication.isPlaying)
            {
                Destroy(rendererCopy);
                foreach (var mat in replacementMaterials) Destroy(mat);
            }
            else
            {
                DestroyImmediate(rendererCopy);
                foreach (var mat in replacementMaterials) DestroyImmediate(mat);
            }
#else
            Destroy(rendererCopy);
            Destroy(replacementMaterial);
            foreach (var mat in replacementMaterials) Destroy(mat);
#endif
        }
    }
}