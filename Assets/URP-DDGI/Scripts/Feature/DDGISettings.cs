using UnityEngine;

namespace DDGIURP
{
    /// <summary>
    /// Which system to use to update the probes. 
    /// 
    /// Raytracing uses hardware-accelerated raytracing, with Acceleration Structures
    /// built by the device itself.
    /// 
    /// VoxelTracing is ideal for weaker platform that don't support hardware-accelerated
    /// raytracing. 
    /// </summary>
    public enum DDGIRaytracingType
    {
        Auto,
        HardwareRaytracing,
        VoxelRaytracing
    }

    /// <summary>
    /// Specify how the probes are going to be Updated.
    /// 
    /// Dynamic updates probes dynamically every frame. 
    /// Probes in view or closer to the camera will have higher priority.
    /// 
    /// BakeByScript updates probes when relevant. Use this if your geometry 
    /// is mostly static, and updates every so often.
    /// </summary>
    public enum DDGIDynamismMode
    {
        Dynamic,
        BakeByScript
    }

    [System.Serializable]
    public class DDGISettings
    {
        [Header("General")]
        public DDGIRaytracingType raytracingType;
        public DDGIDynamismMode dynamismMode;

        [Header("Voxels")]
        [Layer] public int voxelRendererLayer;
        public int voxelCascadeResolution = 256;
        public float voxelBaseCascadeScale = 0.1f;
    }
}