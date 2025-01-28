Shader "ProbeDebugShader"
{
    SubShader
    {
        Tags
        {
            "RenderType" = "Opaque"
            "IgnoreProjector" = "True"
            "UniversalMaterialType" = "Unlit"
            "RenderPipeline" = "UniversalPipeline"
        }

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"  
            #define UNITY_INDIRECT_DRAW_ARGS IndirectDrawIndexedArgs
            #include "UnityIndirect.cginc"
            #include "ProbeIncludes.hlsl"

            Texture2D _MainTexture;
            SamplerState sampler_MainTexture;

            Texture2DArray<float3> _Irradiance;
            SamplerState sampler_Irradiance;
            StructuredBuffer<ProbeData> _probeData;
            uint irrRes;
            uint visRes;
            uint irrResPad;
            uint visResPad;
            uint3 probeDimensions;


            struct Attributes
            {
                float4 positionOS   : POSITION;
                float3 normal : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS  : SV_POSITION;
                float3 normal: NORMAL;
                nointerpolation int3 probeIndex : TEXCOORD0;
            };

            Varyings vert(Attributes v, uint svInstanceID : SV_InstanceID)
            {
                InitIndirectDrawArgs(0);
                Varyings o;
                uint cmdID = GetCommandID(0);
                uint instanceID = GetIndirectInstanceID(svInstanceID);
                ProbeData probeData = _probeData[instanceID];
                
                o.positionCS = TransformWorldToHClip(v.positionOS.xyz * 0.25f + probeData.position);
                o.probeIndex = probeData.index;
                o.normal = v.normal;
                return o;
            }

            float4 frag(Varyings i) : SV_Target
            {
                uint irrWidth, irrHeight, irrLayers;
                _Irradiance.GetDimensions(irrWidth, irrHeight, irrLayers);

                int3 probeIndex = i.probeIndex;
                float2 uv = map_octahedral(normalize(i.normal));

                float2 cacheUV = float2(
                    (probeIndex.x * irrResPad + uv.x * irrRes + 1) / (float)irrWidth, 
                    (probeIndex.y * irrResPad + uv.y * irrRes + 1) / (float)irrHeight
                );

                float3 irradiance = _Irradiance.Sample(sampler_Irradiance, float3(cacheUV, probeIndex.z));

                return float4(irradiance, 1);
            }
            ENDHLSL
        }
    }
}