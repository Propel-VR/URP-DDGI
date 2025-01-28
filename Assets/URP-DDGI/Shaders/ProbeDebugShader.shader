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
            #include "DDGI_Includes.hlsl"

            StructuredBuffer<float3> _positionData;

            struct Attributes
            {
                float4 positionOS   : POSITION;
                float4 normal : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS  : SV_POSITION;
                float3 normal: NORMAL;
                nointerpolation int3 probeIndex : TEXCOORD0;
            }; 

            struct v2f
            {
                float4 pos : SV_POSITION;
                
            };

            Varyings vert(Attributes v, uint svInstanceID : SV_InstanceID)
            {
                InitIndirectDrawArgs(0);
                Varyings o;
                uint cmdID = GetCommandID(0);
                uint instanceID = GetIndirectInstanceID(svInstanceID);
                
                o.positionCS = TransformWorldToHClip(v.positionOS * 0.25f + _positionData[instanceID]);
                o.normal = v.normal;
                return o;
            }

            float4 frag(Varyings i) : SV_Target
            {
                float3 normal = normalize(i.normal);
                return float4(map_octahedral(normal), 0, 1);
            }
            ENDHLSL
        }
    }
}