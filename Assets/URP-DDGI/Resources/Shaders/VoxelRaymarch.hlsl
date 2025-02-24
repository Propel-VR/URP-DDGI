//https://github.com/Arlorean/RaymarchVoxels/blob/main/Assets/RaymarchVoxels/Shaders/RaymarchVoxels.hlsl

float2 intersectAABB(float3 rayOrigin, float3 rayDir) {
    float3 tMin = (-0.5 - rayOrigin) / rayDir;
    float3 tMax = (0.5 - rayOrigin) / rayDir;
    float3 t1 = min(tMin, tMax);
    float3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
	// Clamp the tNear to the camera origin
	if (tNear < 0) {
		tNear = 0;
	}
    return float2(tNear, tFar);
};

float3 rayAABBIntersect(float3 rayOrigin, float3 invRayDir, float3 minAABB, float3 maxAABB) {
  float3 tbot = invRayDir * (minAABB - rayOrigin);
  float3 ttop = invRayDir * (maxAABB - rayOrigin);
  float3 tmin = min(ttop, tbot);
  float3 tmax = max(ttop, tbot);
  float2 traverse = max(tmin.xx, tmin.yz);
  float traverselow = max(traverse.x, traverse.y);
  traverse = min(tmax.xx, tmax.yz);
  float traversehi = min(traverse.x, traverse.y);
  return float3(float(traversehi > max(traverselow, 0.0)), traversehi, traverselow);
}



// Raymarch a 3D voxels texture in an AABB unit cube at the origin
// from a camera origin and direction, both also in object space
// return the color of the pixel on the front face of the cube and the face normal (in object space).
void RaymarchVoxels(
	float3 rayOrigin, float3 rayDir, Texture3D<float4> voxels,  
    out float4 color, out float3 normal, out float depth)
{
	color = 0;
	normal = 0;
	depth = -1;

	float3 p = rayOrigin;
	float3 d = rayDir; // Convert to voxel space
	int3 step = sign(d); // The amount to step in whole voxels based on direction
	float3 invRayDir = 1 / rayDir;
	int3 v = floor(p); // The first voxel where the ray enters - tracks voxels passed through in loop below
	float3 tDelta = abs(invRayDir); // TDelta indicates how far along the ray we must move (in units of t) for the component of such a movement to equal the width of a voxel

	int3 dimensions;
	voxels.GetDimensions(dimensions.x,dimensions.y,dimensions.z);

	float3 aabbIntersect = rayAABBIntersect(rayOrigin, invRayDir, (float3)0, (float3)dimensions); // Get just in front of the front face intersection (back faces should be being rendered)
	
	if(!aabbIntersect.x) {
		return;
	}

	
	

	

	// Initialize tMax - The value of t at which the ray crosses the first voxel boundary for that component
	// https://github.com/DJayalath/RayTracingEngine/blob/master/shader.frag
	float3 tMax = 0;
	if (d.x < 0) {
		tMax.x = (p.x - v.x) * tDelta.x;
	}
	else if (d.x > 0) {
		tMax.x = (v.x + 1.0 - p.x) * tDelta.x;
	}
	if (d.y < 0) {
		tMax.y = (p.y - v.y) * tDelta.y;
	}
	else if (d.y > 0) {
		tMax.y = (v.y + 1.0 - p.y) * tDelta.y;
	}
	if (d.z < 0) {
		tMax.z = (p.z - v.z) * tDelta.z;
	}
	else if (d.z > 0) {
		tMax.z = (v.z + 1.0 - p.z) * tDelta.z;
	}

	float t = 0;

	int maxIterations = dimensions.x + dimensions.y + dimensions.z; // Shouldn't be needed but just in case
	
	[loop]
	for (int i = 0; i < maxIterations; i++) {

		// Look up the exact voxel color in the 3D texture (don't sample)
		color = voxels.Load(int4(v,0));
		depth = t;
		if (color.a > 0.5f) {
			return;
		}

		if (tMax.x < tMax.y) {
			if(tMax.x < tMax.z) {
				v.x = v.x + step.x;
				t = tMax.x;
				if (v.x < 0 || v.x >= dimensions.x) { break; }
				tMax.x += tDelta.x;
				normal = float3(-step.x,0,0);
			} else {
				v.z = v.z + step.z;
				t = tMax.z;
				if (v.z < 0 || v.z >= dimensions.z) { break; }
				tMax.z += tDelta.z;
				normal = float3(0,0,-step.z);
			}
		} else {
			if (tMax.y < tMax.z) {
				v.y = v.y + step.y;
				t = tMax.y;
				if (v.y < 0 || v.y >= dimensions.y) { break; }
				tMax.y += tDelta.y;
				normal = float3(0,-step.y,0);
			} else {
				v.z = v.z + step.z;
				t = tMax.z;
				if (v.z < 0 || v.z >= dimensions.z) { break; }
				tMax.z += tDelta.z;
				normal = float3(0,0,-step.z);
			}
		}
	}

	color = 0;
	normal = 0;
	depth = -1;
}