using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DDGIURP
{
    public class VoxelRendererManager
    {
        public HashSet<VoxelRenderer> renderers = new();
        public HashSet<VoxelRenderer> renderersDynamic = new();
        public Dictionary<Mesh, Mesh> splitMeshes = new();

        static VoxelRendererManager inst;

        // Clean up split meshes
        ~VoxelRendererManager ()
        {
            foreach (var pair in splitMeshes)
            {
#if UNITY_EDITOR
                if (EditorApplication.isPlaying)
                {
                    Object.Destroy(pair.Value);
                }
                else
                {
                    Object.DestroyImmediate(pair.Value);
                }
#else
                Object.Destroy(pair.Value);
#endif
            }
        }

        public static void AddRenderer (VoxelRenderer renderer, bool isStatic)
        {
            // Ensure voxel renderer Manager
            if (inst == null) {
                inst = new VoxelRendererManager();

            }

            inst.renderers.Add(renderer);

            if(!isStatic)
            {
                inst.renderersDynamic.Add(renderer);
            }
        }

        public static void RemoveRenderer (VoxelRenderer renderer)
        {
            if (inst == null) return;

            inst.renderers.Remove(renderer);
            inst.renderersDynamic.Remove(renderer);
        }

        public static Mesh AddMesh (Mesh mesh, bool isSkinned)
        {
            if (inst == null) return null;

            if(inst.splitMeshes.TryGetValue(mesh, out Mesh splitMesh))
            {
                if (splitMesh != null)
                {
                    return splitMesh;
                }
                else
                {
                    inst.splitMeshes.Remove(mesh);
                }
            }

            splitMesh = MeshSplitter.CreateSplitMesh(mesh, isSkinned);
            inst.splitMeshes.Add(mesh, splitMesh);
            return splitMesh;
        }

        public static void RemoveMesh (Mesh mesh)
        {
            if (inst == null) return;

            // Uhhh... Should we do anything?
        }

        // Call in a late update to ensure all renderers replicated their behaviour
        public static void UpdateTransforms ()
        {
            if (inst == null) return;

            foreach (var renderer in inst.renderersDynamic)
            {
                renderer.OnUpdateTransform();
            }
        }
    }
}