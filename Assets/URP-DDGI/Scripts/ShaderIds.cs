using UnityEngine;

namespace DDGIURP
{
    public static class ShaderIds
    {
        public static readonly int
            irrRes = Shader.PropertyToID("irrRes"),
            visRes = Shader.PropertyToID("visRes"),
            irrResPad = Shader.PropertyToID("irrResPad"),
            visResPad = Shader.PropertyToID("visResPad"),
            probeDimensions = Shader.PropertyToID("probeDimensions"),
            _Visibility = Shader.PropertyToID("_Visibility"),
            _Irradiance = Shader.PropertyToID("_Irradiance"),
            _probeData = Shader.PropertyToID("_probeData"),
            _ = Shader.PropertyToID("_");
    }
}