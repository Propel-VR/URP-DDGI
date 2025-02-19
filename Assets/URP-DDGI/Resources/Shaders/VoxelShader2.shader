Shader "Unlit/VoxelShader2"
{
    Properties
    {
        [MainTexture] _BaseMap("Albedo", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalRenderPipeline" }
        Conservative True
        Cull Off
        ZWrite Off
        ZTest Always
        ColorMask 0

        Pass
        {
            HLSLPROGRAM
            #pragma target 5.0
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
            #include "VoxelIncludes.hlsl"

            struct Attributes 
            {
                float3 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : POSITION1;
                float2 uv : TEXCOORD0;
                nointerpolation float targetIndex : TEXCOORD1;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            TEXTURE2D(_BaseMap);
            SAMPLER(sampler_BaseMap);

            CBUFFER_START(UnityPerMaterial)
            float4 _BaseMap_ST;
            CBUFFER_END

            uniform RWTexture3D<float4> _VoxelTarget : register(u1);
            uniform float4x4 ViewProjectionMatrix[3];
            uniform uint resolution;
            uniform float invScale;
            uniform uint VertexStride;
            uniform uint PositionOffset;
            uniform uint UVOffset;
            uniform float4 VolumeCenter;

            Varyings vert (Attributes i)
            {
                Varyings o = (Varyings)0;

                UNITY_SETUP_INSTANCE_ID(i);
                UNITY_TRANSFER_INSTANCE_ID(i, o);

                float3 normal = mul((float3x3)unity_WorldToObject, i.normalOS);
                int axis = (abs(normal.x) > abs(normal.y)) ? 
                  ((abs(normal.x) > abs(normal.z)) ? 0 : 2) : 
                  ((abs(normal.y) > abs(normal.z)) ? 1 : 2);

                float3 positionWS = mul(unity_WorldToObject, float4(i.positionOS, 1)).xyz;

                o.positionCS = mul(ViewProjectionMatrix[axis], float4(positionWS, 1.0));
                o.uv = TRANSFORM_TEX(i.uv, _BaseMap);
                o.targetIndex = axis;
                o.positionWS = positionWS;

                return o;
            }

            half frag (Varyings i) : SV_Target
            {
                half4 col = SAMPLE_TEXTURE2D(_BaseMap, sampler_BaseMap, i.uv);

                int3 coord = round((i.positionWS - VolumeCenter.xyz) * invScale + 0.5f);

                _VoxelTarget[coord] = float4(col.rgb, 1);


                return 0;
            }
            ENDHLSL
        }
    }
}
