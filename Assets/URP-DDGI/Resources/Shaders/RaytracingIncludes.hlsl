#ifndef RAYTRACINGINCLUDES_HLSL
#define RAYTRACINGINCLUDES_HLSL

#include "ProbeIncludes.hlsl"

struct Ray
{
    float3 origin;
    float3 direction;
};

struct RayHit
{
    float t;
    float2 barycentric;
    uint triIndex;
    uint steps;
};

#define TRIANGE_ATTRIBUTE_SIZE 60
struct TriangleAttributes
{
    float3 normal0;
    float3 normal1;
    float3 normal2;
    
    float2 uv0;
    float2 uv1;
    float2 uv2;
};

// Note: Should pack more?
#define TRIANGLE_PACKED_SIZE 48
struct TriangleAttributesPacked
{
    float4 normal0_normal1;
    float4 normal2_uv0;
    float4 uv1_uv2;
};

TriangleAttributesPacked PackTriangles(TriangleAttributes unpacked)
{
    TriangleAttributesPacked packed;
    packed.normal0_normal1 = float4(map_octahedral(unpacked.normal0), map_octahedral(unpacked.normal1));
    packed.normal2_uv0 = float4(map_octahedral(unpacked.normal2), unpacked.uv0);
    packed.uv1_uv2 = float4(unpacked.uv1, unpacked.uv2);
    return packed;
};

TriangleAttributes UnpackTriangles(TriangleAttributesPacked packed)
{
    TriangleAttributes unpacked;
    unpacked.normal0 = unmap_octahedral(packed.normal0_normal1.xy);
    unpacked.normal1 = unmap_octahedral(packed.normal2_uv0.zw);

}



uint TotalRays;
RWStructuredBuffer<Ray> RayBuffer;
RWStructuredBuffer<RayHit> RayHitBuffer;
RWStructuredBuffer<TriangleAttributes> TriangleAttributesBuffer;

float FarPlane;
uint OutputWidth;
uint OutputHeight;
RWTexture2D<float4> Output;

#endif