Shader "Hidden/VoxelRaytraceDebug"
 {
	HLSLINCLUDE

	#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
	#include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"
	#include "VoxelRaymarch.hlsl"

	uniform Texture3D<float4> _VoxelCascade0;
    uniform uint resolution;
    uniform float invScale;
    uniform float4 VolumeCenter;
	uniform float4x4 VP_MATRIX;
	uniform float4x4 INV_VP_MATRIX;
	uniform float3 _CameraWorldPos;
	uniform float3 _VoxelCascadeO_Origin;

	float4 Frag (Varyings input) : SV_Target {
		float2 uv = input.texcoord;
		uv = float2(uv.x, 1-uv.y);
		uv = uv * 2.0 - 1;

		int3 dimensions;
		_VoxelCascade0.GetDimensions(dimensions.x,dimensions.y,dimensions.z);

		float3 rayOrigin = _CameraWorldPos;
		float4 projectedPosition = mul(INV_VP_MATRIX, float4(uv, 1, 1));
		projectedPosition /= projectedPosition.w;
		float3 rayDir = normalize(projectedPosition.xyz - rayOrigin);

		rayOrigin -= _VoxelCascadeO_Origin;
		rayOrigin *= invScale;
		rayOrigin += 1;
		rayOrigin += dimensions*0.5f;

		

		float4 finalColor = 0; 
		float totalWeight = 0;
		float weight = 100.0f;  // Start with full weight

		float4 color; 
		float3 normal; 
		float depth;
        [loop]
		for(int i = 0; i < 1; i++) 
        { 
            RaymarchVoxels(rayOrigin, rayDir, _VoxelCascade0, color, normal, depth);
			color *= (depth >= 0);

			float fresnel = pow(saturate(dot(rayDir, -normal)), 2);
			if(depth < 0) {
				fresnel = 1;
			}
			float bounceWeight = weight * fresnel; // Reduce based on angle

			finalColor += color * bounceWeight;  
			totalWeight += bounceWeight;


			if(depth < 0) {
				break;
			}

			rayOrigin += rayDir * depth + normal * 0.01f;
			rayDir = reflect(rayDir, normal);
			weight *= 0.1f;
        }
		
		if (totalWeight > 0) {
			finalColor /= totalWeight;
		}
		return finalColor;
	}

	ENDHLSL

	SubShader 
	{
		Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline"}
		ZWrite Off
		Cull Off
		ZTest Always
		Blend SrcAlpha OneMinusSrcAlpha
		
		Pass
		{


			Name "Voxel Raytrace Debug"
			HLSLPROGRAM
			#pragma vertex Vert
			#pragma fragment Frag
			ENDHLSL
		}
	}
 }