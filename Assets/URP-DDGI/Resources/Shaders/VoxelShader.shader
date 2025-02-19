Shader "Unlit/VoxelShader"
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
            #pragma multi_compile __ HAS_32_BIT_INDICES
            #pragma multi_compile __ HAS_UVS

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
            #include "VoxelIncludes.hlsl"


            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : POSITION1;
                float2 uv : TEXCOORD0;
                nointerpolation float targetIndex : TEXCOORD1;
            };

            TEXTURE2D(_BaseMap);
            SAMPLER(sampler_BaseMap);
            float4 _BaseMap_ST;

            uniform RWTexture3D<float4> _VoxelTarget : register(u1);
            ByteAddressBuffer VertexBuffer;
            ByteAddressBuffer IndexBuffer;

            float4x4 ViewProjectionMatrix[3];
            uint resolution;
            float invScale;
            uint VertexStride;
            uint PositionOffset;
            uint UVOffset;
            float4x4 _ObjectToWorld;
            float4 VolumeCenter;

            float3 ReadVertexPosition(uint index)
            {
                uint readIndex = (index * VertexStride) + PositionOffset;
                uint3 posData = VertexBuffer.Load3(readIndex);
                return asfloat(posData);
            }

            float2 ReadVertexUV(uint index)
            {
                #ifdef HAS_UVS
                uint readIndex = (index * VertexStride) + UVOffset;
                uint2 uvData = VertexBuffer.Load2(readIndex);
                return asfloat(uvData);
                #else
                return float2(0,0);
                #endif
            }

            // Temporary instancing variables
            static uint3 indicies;
            static float3 posWS[3];
            static uint axis;

            // Ideally only run once per tris, but we can't :(
            void SetupTriangle (uint triIndex) {

                #if HAS_32_BIT_INDICES
                    uint triStride = 3 * 4; // 32-bit index buffer
                    uint byteOffset = triIndex * triStride;
                    indicies = IndexBuffer.Load3(byteOffset);
                #else
                    uint triStride = 3 * 2; // 16-bit index buffer
                    uint byteOffset = triIndex * triStride;
                    uint alignedOffset = byteOffset & ~3;
                    uint2 packedWords = IndexBuffer.Load2(alignedOffset);
                    uint offsetInWord = byteOffset % 4;
    
                    indicies.x = (offsetInWord == 0) ? (packedWords.x & 0xFFFF) : (packedWords.x >> 16);
                    indicies.y = (offsetInWord == 0) ? (packedWords.x >> 16) : (packedWords.y & 0xFFFF);
                    indicies.z = (offsetInWord == 0) ? (packedWords.y & 0xFFFF) : (packedWords.y >> 16);
                #endif

                posWS[0] = mul(_ObjectToWorld, float4(ReadVertexPosition(indicies[0]), 1)).xyz;
                posWS[1] = mul(_ObjectToWorld, float4(ReadVertexPosition(indicies[1]), 1)).xyz;
                posWS[2] = mul(_ObjectToWorld, float4(ReadVertexPosition(indicies[2]), 1)).xyz;

                float3 normal = normalize(cross(posWS[2] - posWS[0], posWS[1] - posWS[0]));
                axis = (abs(normal.x) > abs(normal.y)) ? 
                  ((abs(normal.x) > abs(normal.z)) ? 0 : 2) : 
                  ((abs(normal.y) > abs(normal.z)) ? 1 : 2);
            }

            Varyings vert (uint vertexId : SV_VertexID, uint instanceId : SV_InstanceID)
            {
                uint triIndex = vertexId / 3;
                SetupTriangle(triIndex);
                vertexId -= triIndex * 3;

                uint index = indicies[vertexId];
                float3 positionWS = posWS[vertexId]; 
                float2 uv = ReadVertexUV(index);

                Varyings o;
                o.positionCS = mul(ViewProjectionMatrix[axis], float4(positionWS, 1.0));
                o.uv = TRANSFORM_TEX(uv, _BaseMap);
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
