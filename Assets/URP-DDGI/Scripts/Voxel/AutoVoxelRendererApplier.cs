using Unity.VisualScripting;
using UnityEngine;

namespace DDGIURP
{
    [DefaultExecutionOrder(5)]
    public class AutoVoxelRendererApplier : MonoBehaviour
    {
        public bool onChildren;
        public bool onAllObjects;

        private void Start ()
        {
            if(onChildren)
            {
                var renderers = GetComponentsInChildren<Renderer>();
                foreach(var renderer in renderers)
                {
                    if((renderer is MeshRenderer || renderer is SkinnedMeshRenderer) && !renderer.gameObject.TryGetComponent<VoxelRenderer>(out _))
                    {
                        renderer.AddComponent<VoxelRenderer>();
                    }
                }
            }
            if(onAllObjects)
            {
                var renderers = FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None);
                foreach (var renderer in renderers)
                {
                    if ((renderer is MeshRenderer || renderer is SkinnedMeshRenderer) && !renderer.gameObject.TryGetComponent<VoxelRenderer>(out _))
                    {
                        renderer.AddComponent<VoxelRenderer>();
                    }
                }
            }
        }
    }
}