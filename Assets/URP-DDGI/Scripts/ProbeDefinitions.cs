using Unity.Mathematics;
using UnityEngine.Rendering;

namespace DDGIURP
{
    // NOTE: Generate HLSL turns "SIZE" into a constant, and turn my int3 fields
    // into int4 fields, breaking everything. How do I avoid this?

    //[GenerateHLSL(PackingRules.Exact, needAccessors = false)]
    public struct ProbeData
	{
		public int3 index;
        public float3 position;

        public static readonly int SIZE = sizeof(int) * 3 + sizeof(float) * 3; 
	};
}