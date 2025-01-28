float msign(float v)
{
    return (v.x >= 0.0) ? 1.0 : -1.0;
}

float2 map_octahedral(float3 nor)
{
    const float fac = 1.0f / (abs(nor.x) + abs(nor.y) + abs(nor.z));
    nor.x *= fac;
    nor.y *= fac;
    if (nor.z < 0.0f)
    {
        const float2 temp = nor.xy;
        nor.x = (1.0f - abs(temp.y)) * msign(temp.x);
        nor.y = (1.0f - abs(temp.x)) * msign(temp.y);
    }
    return float2(nor.x, nor.y) * 0.5f + 0.5f;
}

float3 unmap_octahedral(float2 v)
{
    v = v * 2.0f - 1.0f;
    float3 nor = float3(v, 1.0f - abs(v.x) - abs(v.y)); // Rune Stubbe's version,
    float t = max(-nor.z, 0.0f); // much faster than original
    nor.x += (nor.x > 0.0f) ? -t : t; // implementation of this
    nor.y += (nor.y > 0.0f) ? -t : t; // technique
    return normalize(nor);
}

float2 pgi_probe_normal_to_probe_uv(float3 normal)
{
    return map_octahedral(normal);
}

float3 pgi_probe_uv_to_probe_normal(float2 uv)
{
    return unmap_octahedral(uv);
}