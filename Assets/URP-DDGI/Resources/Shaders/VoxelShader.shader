Shader "Hidden/VoxelShader"
{
    Properties
    {
        [MainTexture] _BaseMap("Base Map", 2D) = "white" {}
    }
    SubShader
    {
        Tags {  "RenderType" = "Opaque"  "RenderPipeline" = "UniversalPipeline"  }


        Conservative True
        Cull Off
        ZWrite Off
        ZTest Always
        ColorMask 0

        Pass
        {
            Name "ForwardLit"
            Tags{ "LightMode" = "UniversalForward" }

            HLSLPROGRAM
            #pragma target 5.0
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"

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

            uniform RWTexture3D<float4> _VoxelCascade0 : register(u1);
            uniform float4x4 ViewProjectionMatrix[3];
            uniform uint resolution;
            uniform float invScale;
            uniform float4 VolumeCenter;

            Varyings vert (Attributes i)
            {
                Varyings o;

                UNITY_SETUP_INSTANCE_ID(i);
                UNITY_TRANSFER_INSTANCE_ID(i, o);

                //float3 positionWS = mul(unity_ObjectToWorld, float4(i.positionOS, 1)).xyz;
                //float3 normal = mul((float3x3)unity_ObjectToWorld, i.normalOS);
                
                float3 normal = TransformObjectToWorldNormal(i.normalOS);
                float3 positionWS = TransformObjectToWorld(i.positionOS);

                
                int axis = (abs(normal.x) > abs(normal.y)) ? 
                  ((abs(normal.x) > abs(normal.z)) ? 0 : 2) : 
                  ((abs(normal.y) > abs(normal.z)) ? 1 : 2);


                o.positionCS = mul(ViewProjectionMatrix[axis], float4(positionWS, 1.0));
                //o.positionCS = TransformWorldToHClip(positionWS);
                o.uv = TRANSFORM_TEX(i.uv, _BaseMap);
                o.targetIndex = axis;
                o.positionWS = positionWS;

                return o;
            }

            float frag (Varyings i) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(i);

                half4 col = SAMPLE_TEXTURE2D(_BaseMap, sampler_BaseMap, i.uv);
                int3 coord = round((i.positionWS - VolumeCenter.xyz) * invScale + 0.5f);
                if(col.a > 0.5f) {
                    _VoxelCascade0[coord] = float4(col.rgb, 1);
                }

                return 1;
            }
            ENDHLSL
        }
    }
}