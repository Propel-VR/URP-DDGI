using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace DDGIURP
{
    [BurstCompile]
    public unsafe struct BVHJob : IJobParallelFor
    {
        NativeArray<BVH> BVHs;

        public BVHJob(NativeArray<BVH> bVHs) => BVHs = bVHs;

        public void Execute(int index)
        {
            var bvh = BVHs[index];
            bvh.Build();
            BVHs[index] = bvh;
        }
    }
}