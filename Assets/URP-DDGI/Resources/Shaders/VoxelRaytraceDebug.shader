Shader "Hidden/VoxelRaytraceDebug"
 {
	HLSLINCLUDE

	#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
	#include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"

	Texture3D<uint> _VoxelTarget;
	uint resolution;
	float4x4 VP_MATRIX;

	float4 Frag (Varyings input) : SV_Target {
		float2 uv = input.texcoord - 0.5;

		const float focalLength = 2.5;
		float3 rayDir = mul(VP_MATRIX, normalize(float3(uv, focalLength)));
		float3 rayOrigin = mul(VP_MATRIX, float3(0,0,0)).xyz;


		
		return float4(rayDir, 1);
	}

	ENDHLSL

	SubShader 
	{
		Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline"}
		ZWrite Off
		Cull Off
		ZTest Always
		
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