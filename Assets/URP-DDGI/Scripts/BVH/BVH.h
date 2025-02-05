class BVH : public BVHBase
{
public:
	friend class BVH_GPU;
	friend class BVH_SoA;
	template <int M> friend class MBVH;
	enum BuildFlags : uint
	{
		NONE = 0,			// Default building behavior (binned, SAH-driven).
		FULLSPLIT = 1		// Split as far as possible, even when SAH doesn't agree.
	};
	struct BVHNode
	{
		// 'Traditional' 32-byte BVH node layout, as proposed by Ingo Wald.
		// When aligned to a cache line boundary, two of these fit together.
		bvhvec3 aabbMin; uint leftFirst; // 16 bytes
		bvhvec3 aabbMax; uint triCount;	// 16 bytes, total: 32 bytes
		bool isLeaf() const { return triCount > 0; /* empty BVH leaves do not exist */ }
		float Intersect( const Ray& ray ) const { return BVH::IntersectAABB( ray, aabbMin, aabbMax ); }
		bool Intersect( const bvhvec3& bmin, const bvhvec3& bmax ) const;
		float SurfaceArea() const { return BVH::SA( aabbMin, aabbMax ); }
	};
	BVH( BVHContext ctx = {} ) { layout = LAYOUT_BVH; context = ctx; }
	BVH( const BVH_Verbose& original ) { layout = LAYOUT_BVH; ConvertFrom( original ); }
	BVH( const float4* vertices, const uint primCount ) { layout = LAYOUT_BVH; Build( vertices, primCount ); }
	BVH( const float4slice& vertices ) { layout = LAYOUT_BVH; Build( vertices ); }
	~BVH();
	void ConvertFrom( const BVH_Verbose& original, bool compact = true );
	float SAHCost( const uint nodeIdx = 0 ) const;
	int32_t NodeCount() const;
	int32_t PrimCount( const uint nodeIdx = 0 ) const;
	void Compact();
	void Save( const char* fileName );
	bool Load( const char* fileName, const float4* vertices, const uint primCount );
	bool Load( const char* fileName, const float4* vertices, const uint* indices, const uint primCount );
	bool Load( const char* fileName, const float4slice& vertices, const uint* indices = 0, const uint primCount = 0 );
	void BuildQuick( const float4* vertices, const uint primCount );
	void BuildQuick( const float4slice& vertices );
	void Build( const float4* vertices, const uint primCount );
	void Build( const float4slice& vertices );
	void Build( const float4* vertices, const uint* indices, const uint primCount );
	void Build( const float4slice& vertices, const uint* indices, const uint primCount );
	void Build( BLASInstance* instances, const uint instCount, BVHBase** blasses, const uint blasCount );
	void Build( void (*customGetAABB)(const unsigned, bvhvec3&, bvhvec3&), const uint primCount );
	void BuildHQ( const float4* vertices, const uint primCount );
	void BuildHQ( const float4slice& vertices );
	void BuildHQ( const float4* vertices, const uint* indices, const uint primCount );
	void BuildHQ( const float4slice& vertices, const uint* indices, const uint primCount );
#ifdef BVH_USEAVX
	void BuildAVX( const float4* vertices, const uint primCount );
	void BuildAVX( const float4slice& vertices );
	void BuildAVX( const float4* vertices, const uint* indices, const uint primCount );
	void BuildAVX( const float4slice& vertices, const uint* indices, const uint primCount );
#endif
	void Refit( const uint nodeIdx = 0 );
	int32_t Intersect( Ray& ray ) const;
	bool IntersectSphere( const bvhvec3& pos, const float r ) const;
	bool IsOccluded( const Ray& ray ) const;
	void Intersect256Rays( Ray* first ) const;
	void Intersect256RaysSSE( Ray* packet ) const; // requires BVH_USEAVX
private:
	void PrepareBuild( const float4slice& vertices, const uint* indices, const uint primCount );
	void Build();
	bool IsOccludedTLAS( const Ray& ray ) const;
	int32_t IntersectTLAS( Ray& ray ) const;
	void PrepareAVXBuild( const float4slice& vertices, const uint* indices, const uint primCount );
	void BuildAVX();
	void PrepareHQBuild( const float4slice& vertices, const uint* indices, const uint prims );
	void BuildHQ();
	bool ClipFrag( const Fragment& orig, Fragment& newFrag, bvhvec3 bmin, bvhvec3 bmax, bvhvec3 minDim, const uint splitAxis );
	void SplitFrag( const Fragment& orig, Fragment& left, Fragment& right, const bvhvec3& minDim, const uint splitAxis, const float splitPos, bool& leftOK, bool& rightOK );
	void RefitUpVerbose( uint nodeIdx );
	uint FindBestNewPosition( const uint Lid );
	void ReinsertNodeVerbose( const uint Lid, const uint Nid, const uint origin );
	uint CountSubtreeTris( const uint nodeIdx, uint* counters );
	void MergeSubtree( const uint nodeIdx, uint* newIdx, uint& newIdxPtr );
protected:
	void BuildDefault( const float4* vertices, const uint primCount );
	void BuildDefault( const float4slice& vertices );
	void BuildDefault( const float4* vertices, const uint* indices, const uint primCount );
	void BuildDefault( const float4slice& vertices, const uint* indices, const uint primCount );
public:
	// BVH type identification
	bool isTLAS() const { return instList != 0; }
	bool isBLAS() const { return instList == 0; }
	bool isIndexed() const { return vertIdx != 0; }
	bool hasCustomGeom() const { return customIntersect != 0; }
	// Basic BVH data
	float4slice verts = {};		// pointer to input primitive array: 3x16 bytes per tri.
	uint* vertIdx = 0;			// vertex indices, only used in case the BVH is built over indexed prims.
	uint* primIdx = 0;			// primitive index array.
	BLASInstance* instList = 0;		// instance array, for top-level acceleration structure.
	BVHBase** blasList = 0;			// blas array, for TLAS traversal.
	uint blasCount = 0;			// number of blasses in blasList.
	BVHNode* bvhNode = 0;			// BVH node pool, Wald 32-byte format. Root is always in node 0.
	uint newNodePtr = 0;		// used during build to keep track of next free node in pool.
	Fragment* fragment = 0;			// input primitive bounding boxes.
	// Custom geometry intersection callback
	bool (*customIntersect)(Ray&, const unsigned) = 0;
	bool (*customIsOccluded)(const Ray&, const unsigned) = 0;
};




BVH::~BVH()
{
	AlignedFree( bvhNode );
	AlignedFree( primIdx );
	AlignedFree( fragment );
}

void BVH::Save( const char* fileName )
{
	// saving is easy, it's the loadingn that will be complex.
	std::fstream s{ fileName, s.binary | s.out };
	uint header = TINY_BVH_VERSION_SUB + (TINY_BVH_VERSION_MINOR << 8) + (TINY_BVH_VERSION_MAJOR << 16) + (layout << 24);
	s.write( (char*)&header, sizeof( uint ) );
	s.write( (char*)&triCount, sizeof( uint ) );
	s.write( (char*)this, sizeof( BVH ) );
	s.write( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.write( (char*)primIdx, idxCount * sizeof( uint ) );
}

bool BVH::Load( const char* fileName, const float4* vertices, const uint primCount )
{
	return Load( fileName, float4slice{ vertices, primCount * 3, sizeof( float4 ) } );
}

bool BVH::Load( const char* fileName, const float4* vertices, const uint* indices, const uint primCount )
{
	return Load( fileName, float4slice{ vertices, primCount * 3, sizeof( float4 ) }, indices, primCount );
}

bool BVH::Load( const char* fileName, const float4slice& vertices, const uint* indices, const uint primCount )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	bool expectIndexed = (indices != nullptr);
	uint header, fileTriCount;
	s.read( (char*)&header, sizeof( uint ) );
	if (((header >> 8) & 255) != TINY_BVH_VERSION_MINOR ||
		((header >> 16) & 255) != TINY_BVH_VERSION_MAJOR ||
		(header & 255) != TINY_BVH_VERSION_SUB || (header >> 24) != layout) return false;
	s.read( (char*)&fileTriCount, sizeof( uint ) );
	if (expectIndexed && fileTriCount != primCount) return false;
	if (!expectIndexed && fileTriCount != vertices.count / 3) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH ) );
	bool fileIsIndexed = vertIdx != nullptr;
	if (expectIndexed != fileIsIndexed) return false; // not what we expected.
	if (blasList != nullptr || instList != nullptr) return false; // can't load/save TLAS.
	context = tmp; // can't load context; function pointers will differ.
	bvhNode = (BVHNode*)AlignedAlloc( allocatedNodes * sizeof( BVHNode ) );
	primIdx = (uint*)AlignedAlloc( idxCount * sizeof( uint ) );
	fragment = 0; // no need for this in a BVH that can't be rebuilt.
	s.read( (char*)bvhNode, usedNodes * sizeof( BVHNode ) );
	s.read( (char*)primIdx, idxCount * sizeof( uint ) );
	verts = vertices; // we can't load vertices since the BVH doesn't own this data.
	vertIdx = (uint*)indices;
	// all ok.
	return true;
}

void BVH::BuildDefault( const float4* vertices, const uint primCount )
{
	// access point for builds over a raw list of vertices. The stride of
	// the vertex data must be 16 bytes.
	// The list will be encapsulated in a 'slice'. The slice can also be used
	// directly to provide data with a different stride, which enables use of
	// vertex buffers created for rasterization.
	BuildDefault( float4slice{ vertices, primCount * 3, sizeof( float4 ) } );
}
void BVH::BuildDefault( const float4* vertices, const uint* indices, const uint primCount )
{
	// access point for builders over an indexed list of raw vertices.
	BuildDefault( float4slice{ vertices, primCount * 3, sizeof( float4 ) }, indices, primCount );
}
void BVH::BuildDefault( const float4slice& vertices )
{
	// default builder: used internally when constructing a BVH layout requires
	// a regular BVH. Currently, this is the case for all of them.
#if defined(BVH_USEAVX)
	// if AVX is supported, BuildAVX is the optimal option. Tree quality is
	// identical to the reference builder, but speed is much better.
	BuildAVX( vertices );
#else
	// fallback option, in case AVX is not supported: the reference builder, which
	// should work on all platforms.
	Build( vertices );
#endif
}
void BVH::BuildDefault( const float4slice& vertices, const uint* indices, const uint primCount )
{
	// default builder for indexed vertices. See notes above.
#if defined(BVH_USEAVX)
	BuildAVX( vertices, indices, primCount );
#else
	Build( vertices, indices, primCount );
#endif
}

void BVH::ConvertFrom( const BVH_Verbose& original, bool compact )
{
	// allocate space
	const uint spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		bvhNode = (BVHNode*)AlignedAlloc( triCount * 2 * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( bvhNode, 0, sizeof( BVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	this->verts = original.verts;
	this->primIdx = original.primIdx;
	// start conversion
	uint srcNodeIdx = 0, dstNodeIdx = 0;
	newNodePtr = 2;
	uint srcStack[64], dstStack[64], stackPtr = 0;
	while (1)
	{
		const BVH_Verbose::BVHNode& orig = original.bvhNode[srcNodeIdx];
		bvhNode[dstNodeIdx].aabbMin = orig.aabbMin;
		bvhNode[dstNodeIdx].aabbMax = orig.aabbMax;
		if (orig.isLeaf())
		{
			bvhNode[dstNodeIdx].triCount = orig.triCount;
			bvhNode[dstNodeIdx].leftFirst = orig.firstTri;
			if (stackPtr == 0) break;
			srcNodeIdx = srcStack[--stackPtr];
			dstNodeIdx = dstStack[stackPtr];
		}
		else
		{
			bvhNode[dstNodeIdx].leftFirst = newNodePtr;
			uint srcRightIdx = orig.right;
			srcNodeIdx = orig.left, dstNodeIdx = newNodePtr++;
			srcStack[stackPtr] = srcRightIdx;
			dstStack[stackPtr++] = newNodePtr++;
		}
	}
	usedNodes = original.usedNodes;
}

float BVH::SAHCost( const uint nodeIdx ) const
{
	// Determine the SAH cost of the tree. This provides an indication
	// of the quality of the BVH: Lower is better.
	const BVHNode& n = bvhNode[nodeIdx];
	if (n.isLeaf()) return C_INT * n.SurfaceArea() * n.triCount;
	float cost = C_TRAV * n.SurfaceArea() + SAHCost( n.leftFirst ) + SAHCost( n.leftFirst + 1 );
	return nodeIdx == 0 ? (cost / n.SurfaceArea()) : cost;
}

int32_t BVH::PrimCount( const uint nodeIdx ) const
{
	// Determine the total number of primitives / fragments in leaf nodes.
	const BVHNode& n = bvhNode[nodeIdx];
	return n.isLeaf() ? n.triCount : (PrimCount( n.leftFirst ) + PrimCount( n.leftFirst + 1 ));
}

// Basic single-function BVH builder, using mid-point splits.
// This builder yields a correct BVH in little time, but the quality of the
// structure will be low. Use this only if build time is the bottleneck in
// your application (e.g., when you need to trace few rays).
void BVH::BuildQuick( const float4* vertices, const uint primCount )
{
	// build the BVH with a continuous array of float4 vertices:
	// in this case, the stride for the slice is 16 bytes.
	BuildQuick( float4slice{ vertices, primCount * 3, sizeof( float4 ) } );
}
void BVH::BuildQuick( const float4slice& vertices )
{
	FATAL_ERROR_IF( vertices.count == 0, "BVH::BuildQuick( .. ), primCount == 0." );
	// allocate on first build
	const uint primCount = vertices.count / 3;
	const uint spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint*)AlignedAlloc( primCount * sizeof( uint ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::BuildQuick( .. ), bvh not rebuildable." );
	verts = vertices; // note: we're not copying this data; don't delete.
	idxCount = triCount = primCount;
	// reset node pool
	newNodePtr = 2;
	// assign all triangles to the root node
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	// initialize fragments and initialize root node bounds
	for (uint i = 0; i < triCount; i++)
	{
		fragment[i].bmin = tinybvh_min( tinybvh_min( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		fragment[i].bmax = tinybvh_max( tinybvh_max( verts[i * 3], verts[i * 3 + 1] ), verts[i * 3 + 2] );
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
	}
	// subdivide recursively
	uint task[256], taskCount = 0, nodeIdx = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// in-place partition against midpoint on longest axis
			uint j = node.leftFirst + node.triCount, src = node.leftFirst;
			bvhvec3 extent = node.aabbMax - node.aabbMin;
			uint axis = 0;
			if (extent.y > extent.x && extent.y > extent.z) axis = 1;
			if (extent.z > extent.x && extent.z > extent.y) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f, centroid;
			bvhvec3 lbmin( BVH_FAR ), lbmax( -BVH_FAR ), rbmin( BVH_FAR ), rbmax( -BVH_FAR ), fmin, fmax;
			for (uint fi, i = 0; i < node.triCount; i++)
			{
				fi = primIdx[src], fmin = fragment[fi].bmin, fmax = fragment[fi].bmax;
				centroid = (fmin[axis] + fmax[axis]) * 0.5f;
				if (centroid < splitPos)
					lbmin = tinybvh_min( lbmin, fmin ), lbmax = tinybvh_max( lbmax, fmax ), src++;
				else
				{
					rbmin = tinybvh_min( rbmin, fmin ), rbmax = tinybvh_max( rbmax, fmax );
					tinybvh_swap( primIdx[src], primIdx[--j] );
				}
			}
			// create child nodes
			const uint leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // split did not work out.
			const int32_t lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = lbmin, bvhNode[lci].aabbMax = lbmax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = rbmin, bvhNode[rci].aabbMax = rbmax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			// recurse
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
	usedNodes = newNodePtr;
}

// Basic single-function binned-SAH-builder.
// This is the reference builder; it yields a decent tree suitable for ray tracing on the CPU.
// This code uses no SIMD instructions. Faster code, using SSE/AVX, is available for x64 CPUs.
// For GPU rendering: The resulting BVH should be converted to a more optimal
// format after construction, e.g. BVH_GPU, BVH4_GPU or BVH8_CWBVH.
void BVH::Build( const float4* vertices, const uint prims )
{
	// build the BVH with a continuous array of float4 vertices:
	// in this case, the stride for the slice is 16 bytes.
	Build( float4slice{ vertices, prims * 3, sizeof( float4 ) } );
}
void BVH::Build( const float4slice& vertices )
{
	// build the BVH from vertices stored in a slice.
	PrepareBuild( vertices, 0, 0 /* empty index list; primcount is derived from slice */ );
	Build();
}
void BVH::Build( const float4* vertices, const uint* indices, const uint prims )
{
	// build the BVH with a continuous array of float4 vertices, indexed by 'indices'.
	Build( float4slice{ vertices, prims * 3, sizeof( float4 ) }, indices, prims );
}
void BVH::Build( const float4slice& vertices, const uint* indices, uint prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	PrepareBuild( vertices, indices, prims );
	Build();
}

void BVH::Build( void (*customGetAABB)(const unsigned, bvhvec3&, bvhvec3&), const uint primCount )
{
	FATAL_ERROR_IF( primCount == 0, "BVH::Build( void (*customGetAABB)( .. ), instCount ), instCount == 0." );
	triCount = idxCount = primCount;
	const uint spaceNeeded = primCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint*)AlignedAlloc( primCount * sizeof( uint ) );
		fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
	}
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = primCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	for (uint i = 0; i < primCount; i++)
	{
		customGetAABB( i, fragment[i].bmin, fragment[i].bmax );
		fragment[i].primIdx = i, fragment[i].clipped = 0, primIdx[i] = i;
		root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
		root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax );
	}
	// start build
	newNodePtr = 2;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH::Build( BLASInstance* instances, const uint instCount, BVHBase** blasses, const uint bCount )
{
	FATAL_ERROR_IF( instCount == 0, "BVH::Build( BLASInstance*, instCount ), instCount == 0." );
	triCount = idxCount = instCount;
	const uint spaceNeeded = instCount * 2; // upper limit
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint*)AlignedAlloc( instCount * sizeof( uint ) );
		fragment = (Fragment*)AlignedAlloc( instCount * sizeof( Fragment ) );
	}
	instList = instances;
	blasList = blasses;
	blasCount = bCount;
	// copy relevant data from instance array
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = instCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	for (uint i = 0; i < instCount; i++)
	{
		if (blasList) // if a null pointer is passed, we'll assume the BLASInstances have been updated elsewhere.
		{
			uint blasIdx = instList[i].blasIdx;
			BVH* blas = (BVH*)blasList[blasIdx];
			instList[i].Update( blas );
		}
		fragment[i].bmin = instList[i].aabbMin, fragment[i].primIdx = i;
		fragment[i].bmax = instList[i].aabbMax, fragment[i].clipped = 0;
		root.aabbMin = tinybvh_min( root.aabbMin, instList[i].aabbMin );
		root.aabbMax = tinybvh_max( root.aabbMax, instList[i].aabbMax ), primIdx[i] = i;
	}
	// start build
	newNodePtr = 2;
	Build(); // or BuildAVX, for large TLAS.
}

void BVH::PrepareBuild( const float4slice& vertices, const uint* indices, const uint prims )
{
	uint primCount = prims > 0 ? prims : vertices.count / 3;
	const uint spaceNeeded = primCount * 2; // upper limit
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint*)AlignedAlloc( primCount * sizeof( uint ) );
		if (vertices) fragment = (Fragment*)AlignedAlloc( primCount * sizeof( Fragment ) );
		else FATAL_ERROR_IF( fragment == 0, "BVH::PrepareBuild( 0, .. ), not called from ::Build( aabb )." );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::PrepareBuild( .. ), bvh not rebuildable." );
	verts = vertices, idxCount = triCount = primCount, vertIdx = (uint*)indices;
	// prepare fragments
	FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareBuild( .. ), empty vertex slice." );
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	if (!indices)
	{
		FATAL_ERROR_IF( prims != 0, "BVH::PrepareBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (uint i = 0; i < triCount; i++)
		{
			const float4 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const float4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const float4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		FATAL_ERROR_IF( prims == 0, "BVH::PrepareBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (uint i = 0; i < triCount; i++)
		{
			const uint i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const float4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const float4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const float4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// reset node pool
	newNodePtr = 2;
	bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::Build.
}
void BVH::Build()
{
	// subdivide root node recursively
	uint task[256], taskCount = 0, nodeIdx = 0;
	BVHNode& root = bvhNode[0];
	bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-20f, bestLMin = 0, bestLMax = 0, bestRMin = 0, bestRMax = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][BVHBINS], binMax[3][BVHBINS];
			for (uint a = 0; a < 3; a++) for (uint i = 0; i < BVHBINS; i++) binMin[a][i] = BVH_FAR, binMax[a][i] = -BVH_FAR;
			uint count[3][BVHBINS];
			memset( count, 0, BVHBINS * 3 * sizeof( uint ) );
			const bvhvec3 rpd3 = bvhvec3( BVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint fi = primIdx[node.leftFirst + i];
				bvhint3 bi = bvhint3( ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3 );
				bi.x = tinybvh_clamp( bi.x, 0, BVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, BVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, BVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			float splitCost = BVH_FAR, rSAV = 1.0f / node.SurfaceArea();
			uint bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim[a])
			{
				bvhvec3 lBMin[BVHBINS - 1], rBMin[BVHBINS - 1], l1 = BVH_FAR, l2 = -BVH_FAR;
				bvhvec3 lBMax[BVHBINS - 1], rBMax[BVHBINS - 1], r1 = BVH_FAR, r2 = -BVH_FAR;
				float ANL[BVHBINS - 1], ANR[BVHBINS - 1];
				for (uint lN = 0, rN = 0, i = 0; i < BVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[BVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][BVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[BVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][BVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][BVHBINS - 1 - i];
					ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
					ANR[BVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint i = 0; i < BVHBINS - 1; i++)
				{
					const float C = C_TRAV + rSAV * C_INT * (ANL[i] + ANR[i]);
					if (C < splitCost)
					{
						splitCost = C, bestAxis = a, bestPos = i;
						bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
					}
				}
			}
			float noSplitCost = (float)node.triCount * C_INT;
			if (splitCost >= noSplitCost) break; // not splitting is better.
			// in-place partition
			uint j = node.leftFirst + node.triCount, src = node.leftFirst;
			const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
			for (uint i = 0; i < node.triCount; i++)
			{
				const uint fi = primIdx[src];
				int32_t bi = (uint)(((fragment[fi].bmin[bestAxis] + fragment[fi].bmax[bestAxis]) * 0.5f - nmin) * rpd);
				bi = tinybvh_clamp( bi, 0, BVHBINS - 1 );
				if ((uint)bi <= bestPos) src++; else tinybvh_swap( primIdx[src], primIdx[--j] );
			}
			// create child nodes
			uint leftCount = src - node.leftFirst, rightCount = node.triCount - leftCount;
			if (leftCount == 0 || rightCount == 0) break; // should not happen.
			const int32_t lci = newNodePtr++, rci = newNodePtr++;
			bvhNode[lci].aabbMin = bestLMin, bvhNode[lci].aabbMax = bestLMax;
			bvhNode[lci].leftFirst = node.leftFirst, bvhNode[lci].triCount = leftCount;
			bvhNode[rci].aabbMin = bestRMin, bvhNode[rci].aabbMax = bestRMax;
			bvhNode[rci].leftFirst = j, bvhNode[rci].triCount = rightCount;
			node.leftFirst = lci, node.triCount = 0;
			// recurse
			task[taskCount++] = rci, nodeIdx = lci;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else nodeIdx = task[--taskCount];
	}
	// all done.
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = true; // not using spatial splits: can refit this BVH
	may_have_holes = false; // the reference builder produces a continuous list of nodes
	bvh_over_aabbs = (verts == 0); // bvh over aabbs is suitable as TLAS
	usedNodes = newNodePtr;
}

// SBVH builder.
// Besides the regular object splits used in the reference builder, the SBVH
// algorithm also considers spatial splits, where primitives may be cut in
// multiple parts. This increases primitive count but may reduce overlap of
// BVH nodes. The cost of each option is considered per split.
// For typical geometry, SBVH yields a tree that can be traversed 25% faster.
// This comes at greatly increased construction cost, making the SBVH
// primarily useful for static geometry.
void BVH::BuildHQ( const float4* vertices, const uint primCount )
{
	BuildHQ( float4slice{ vertices, primCount * 3, sizeof( float4 ) } );
}

void BVH::BuildHQ( const float4* vertices, const uint* indices, const uint prims )
{
	// build the BVH with a continuous array of float4 vertices, indexed by 'indices'.
	BuildHQ( float4slice{ vertices, prims * 3, sizeof( float4 ) }, indices, prims );
}

void BVH::BuildHQ( const float4slice& vertices )
{
	PrepareHQBuild( vertices, 0, 0 );
	BuildHQ();
}

void BVH::BuildHQ( const float4slice& vertices, const uint* indices, uint prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	PrepareHQBuild( vertices, indices, prims );
	BuildHQ();
}

void BVH::PrepareHQBuild( const float4slice& vertices, const uint* indices, const uint prims )
{
	uint primCount = prims > 0 ? prims : vertices.count / 3;
	const uint slack = primCount >> 1; // for split prims
	const uint spaceNeeded = primCount * 3;
	// allocate memory on first build
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( bvhNode );
		AlignedFree( primIdx );
		AlignedFree( fragment );
		bvhNode = (BVHNode*)AlignedAlloc( spaceNeeded * sizeof( BVHNode ) );
		allocatedNodes = spaceNeeded;
		memset( &bvhNode[1], 0, 32 );	// node 1 remains unused, for cache line alignment.
		primIdx = (uint*)AlignedAlloc( (primCount + slack) * sizeof( uint ) );
		fragment = (Fragment*)AlignedAlloc( (primCount + slack) * sizeof( Fragment ) );
	}
	else FATAL_ERROR_IF( !rebuildable, "BVH::PrepareHQBuild( .. ), bvh not rebuildable." );
	verts = vertices; // note: we're not copying this data; don't delete.
	idxCount = primCount + slack, triCount = primCount, vertIdx = (uint*)indices;
	// prepare fragments
	BVHNode& root = bvhNode[0];
	root.leftFirst = 0, root.triCount = triCount, root.aabbMin = bvhvec3( BVH_FAR ), root.aabbMax = bvhvec3( -BVH_FAR );
	if (!indices)
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), primCount == 0." );
		FATAL_ERROR_IF( prims != 0, "BVH::PrepareHQBuild( .. ), indices == 0." );
		// building a BVH over triangles specified as three 16-byte vertices each.
		for (uint i = 0; i < triCount; i++)
		{
			const float4 v0 = verts[i * 3], v1 = verts[i * 3 + 1], v2 = verts[i * 3 + 2];
			const float4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const float4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	else
	{
		FATAL_ERROR_IF( vertices.count == 0, "BVH::PrepareHQBuild( .. ), empty vertex slice." );
		FATAL_ERROR_IF( prims == 0, "BVH::PrepareHQBuild( .. ), prims == 0." );
		// building a BVH over triangles consisting of vertices indexed by 'indices'.
		for (uint i = 0; i < triCount; i++)
		{
			const uint i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
			const float4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
			const float4 fmin = tinybvh_min( v0, tinybvh_min( v1, v2 ) );
			const float4 fmax = tinybvh_max( v0, tinybvh_max( v1, v2 ) );
			fragment[i].bmin = fmin, fragment[i].bmax = fmax, fragment[i].primIdx = i, fragment[i].clipped = 0;
			root.aabbMin = tinybvh_min( root.aabbMin, fragment[i].bmin );
			root.aabbMax = tinybvh_max( root.aabbMax, fragment[i].bmax ), primIdx[i] = i;
		}
	}
	// clear remainder of index array
	memset( primIdx + triCount, 0, slack * 4 );
	bvh_over_indices = indices != nullptr;
	// all set; actual build happens in BVH::Build.
}
void BVH::BuildHQ()
{
	const uint slack = triCount >> 1; // for split prims
	uint* triIdxA = primIdx;
	uint* triIdxB = (uint*)AlignedAlloc( (triCount + slack) * sizeof( uint ) );
	memset( triIdxB, 0, (triCount + slack) * 4 );
	// reset node pool
	newNodePtr = 2;
	uint nextFrag = triCount;
	// subdivide recursively
	BVHNode& root = bvhNode[0];
	const float rootArea = (root.aabbMax - root.aabbMin).halfArea();
	struct Task { uint node, sliceStart, sliceEnd, dummy; };
	ALIGNED( 64 ) Task task[1024];
	uint taskCount = 0, nodeIdx = 0, sliceStart = 0, sliceEnd = triCount + slack;
	const bvhvec3 minDim = (root.aabbMax - root.aabbMin) * 1e-7f /* don't touch, carefully picked */;
	bvhvec3 bestLMin = 0, bestLMax = 0, bestRMin = 0, bestRMax = 0;
	while (1)
	{
		while (1)
		{
			BVHNode& node = bvhNode[nodeIdx];
			// find optimal object split
			bvhvec3 binMin[3][HQBVHBINS], binMax[3][HQBVHBINS];
			for (uint a = 0; a < 3; a++) for (uint i = 0; i < HQBVHBINS; i++) binMin[a][i] = BVH_FAR, binMax[a][i] = -BVH_FAR;
			uint count[3][HQBVHBINS];
			memset( count, 0, HQBVHBINS * 3 * sizeof( uint ) );
			const bvhvec3 rpd3 = bvhvec3( HQBVHBINS / (node.aabbMax - node.aabbMin) ), nmin3 = node.aabbMin;
			for (uint i = 0; i < node.triCount; i++) // process all tris for x,y and z at once
			{
				const uint fi = primIdx[node.leftFirst + i];
				bvhint3 bi = bvhint3( ((fragment[fi].bmin + fragment[fi].bmax) * 0.5f - nmin3) * rpd3 );
				bi.x = tinybvh_clamp( bi.x, 0, HQBVHBINS - 1 );
				bi.y = tinybvh_clamp( bi.y, 0, HQBVHBINS - 1 );
				bi.z = tinybvh_clamp( bi.z, 0, HQBVHBINS - 1 );
				binMin[0][bi.x] = tinybvh_min( binMin[0][bi.x], fragment[fi].bmin );
				binMax[0][bi.x] = tinybvh_max( binMax[0][bi.x], fragment[fi].bmax ), count[0][bi.x]++;
				binMin[1][bi.y] = tinybvh_min( binMin[1][bi.y], fragment[fi].bmin );
				binMax[1][bi.y] = tinybvh_max( binMax[1][bi.y], fragment[fi].bmax ), count[1][bi.y]++;
				binMin[2][bi.z] = tinybvh_min( binMin[2][bi.z], fragment[fi].bmin );
				binMax[2][bi.z] = tinybvh_max( binMax[2][bi.z], fragment[fi].bmax ), count[2][bi.z]++;
			}
			// calculate per-split totals
			float splitCost = 1e30f, rSAV = 1.0f / node.SurfaceArea();
			uint bestAxis = 0, bestPos = 0;
			for (int32_t a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
			{
				bvhvec3 lBMin[HQBVHBINS - 1], rBMin[HQBVHBINS - 1], l1 = BVH_FAR, l2 = -BVH_FAR;
				bvhvec3 lBMax[HQBVHBINS - 1], rBMax[HQBVHBINS - 1], r1 = BVH_FAR, r2 = -BVH_FAR;
				float ANL[HQBVHBINS - 1], ANR[HQBVHBINS - 1];
				for (uint lN = 0, rN = 0, i = 0; i < HQBVHBINS - 1; i++)
				{
					lBMin[i] = l1 = tinybvh_min( l1, binMin[a][i] );
					rBMin[HQBVHBINS - 2 - i] = r1 = tinybvh_min( r1, binMin[a][HQBVHBINS - 1 - i] );
					lBMax[i] = l2 = tinybvh_max( l2, binMax[a][i] );
					rBMax[HQBVHBINS - 2 - i] = r2 = tinybvh_max( r2, binMax[a][HQBVHBINS - 1 - i] );
					lN += count[a][i], rN += count[a][HQBVHBINS - 1 - i];
					ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
					ANR[HQBVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
				}
				// evaluate bin totals to find best position for object split
				for (uint i = 0; i < HQBVHBINS - 1; i++)
				{
					const float C = C_TRAV + C_INT * rSAV * (ANL[i] + ANR[i]);
					if (C >= splitCost) continue;
					splitCost = C, bestAxis = a, bestPos = i;
					bestLMin = lBMin[i], bestRMin = rBMin[i], bestLMax = lBMax[i], bestRMax = rBMax[i];
				}
			}
			// consider a spatial split
			bool spatial = false;
			uint NL[HQBVHBINS - 1], NR[HQBVHBINS - 1], budget = sliceEnd - sliceStart, bestNL = 0, bestNR = 0;
			bvhvec3 spatialUnion = bestLMax - bestRMin;
			float spatialOverlap = (spatialUnion.halfArea()) / rootArea;
			if (budget > node.triCount && splitCost < 1e30f && spatialOverlap > 1e-5f)
			{
				for (uint a = 0; a < 3; a++) if ((node.aabbMax[a] - node.aabbMin[a]) > minDim.cell[a])
				{
					// setup bins
					bvhvec3 binaMin[HQBVHBINS], binaMax[HQBVHBINS];
					for (uint i = 0; i < HQBVHBINS; i++) binaMin[i] = BVH_FAR, binaMax[i] = -BVH_FAR;
					uint countIn[HQBVHBINS] = { 0 }, countOut[HQBVHBINS] = { 0 };
					// populate bins with clipped fragments
					const float planeDist = (node.aabbMax[a] - node.aabbMin[a]) / (HQBVHBINS * 0.9999f);
					const float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[a];
					for (uint i = 0; i < node.triCount; i++)
					{
						const uint fragIdx = triIdxA[node.leftFirst + i];
						const int32_t bin1 = tinybvh_clamp( (int32_t)((fragment[fragIdx].bmin[a] - nodeMin) * rPlaneDist), 0, HQBVHBINS - 1 );
						const int32_t bin2 = tinybvh_clamp( (int32_t)((fragment[fragIdx].bmax[a] - nodeMin) * rPlaneDist), 0, HQBVHBINS - 1 );
						countIn[bin1]++, countOut[bin2]++;
						if (bin2 == bin1) // fragment fits in a single bin
							binaMin[bin1] = tinybvh_min( binaMin[bin1], fragment[fragIdx].bmin ),
							binaMax[bin1] = tinybvh_max( binaMax[bin1], fragment[fragIdx].bmax );
						else for (int32_t j = bin1; j <= bin2; j++)
						{
							// clip fragment to each bin it overlaps
							bvhvec3 bmin = node.aabbMin, bmax = node.aabbMax;
							bmin[a] = nodeMin + planeDist * j;
							bmax[a] = j == (HQBVHBINS - 2) ? node.aabbMax[a] : (bmin[a] + planeDist);
							Fragment orig = fragment[fragIdx];
							Fragment tmpFrag;
							if (!ClipFrag( orig, tmpFrag, bmin, bmax, minDim, a )) continue;
							binaMin[j] = tinybvh_min( binaMin[j], tmpFrag.bmin );
							binaMax[j] = tinybvh_max( binaMax[j], tmpFrag.bmax );
						}
					}
					// evaluate split candidates
					bvhvec3 lBMin[HQBVHBINS - 1], rBMin[HQBVHBINS - 1], l1 = BVH_FAR, l2 = -BVH_FAR;
					bvhvec3 lBMax[HQBVHBINS - 1], rBMax[HQBVHBINS - 1], r1 = BVH_FAR, r2 = -BVH_FAR;
					float ANL[HQBVHBINS], ANR[HQBVHBINS];
					for (uint lN = 0, rN = 0, i = 0; i < HQBVHBINS - 1; i++)
					{
						lBMin[i] = l1 = tinybvh_min( l1, binaMin[i] ), rBMin[HQBVHBINS - 2 - i] = r1 = tinybvh_min( r1, binaMin[HQBVHBINS - 1 - i] );
						lBMax[i] = l2 = tinybvh_max( l2, binaMax[i] ), rBMax[HQBVHBINS - 2 - i] = r2 = tinybvh_max( r2, binaMax[HQBVHBINS - 1 - i] );
						lN += countIn[i], rN += countOut[HQBVHBINS - 1 - i], NL[i] = lN, NR[HQBVHBINS - 2 - i] = rN;
						ANL[i] = lN == 0 ? BVH_FAR : ((l2 - l1).halfArea() * (float)lN);
						ANR[HQBVHBINS - 2 - i] = rN == 0 ? BVH_FAR : ((r2 - r1).halfArea() * (float)rN);
					}
					// find best position for spatial split
					for (uint i = 0; i < HQBVHBINS - 1; i++)
					{
						const float Cspatial = C_TRAV + C_INT * rSAV * (ANL[i] + ANR[i]);
						if (Cspatial < splitCost && NL[i] + NR[i] < budget)
						{
							spatial = true, splitCost = Cspatial, bestAxis = a, bestPos = i;
							bestLMin = lBMin[i], bestLMax = lBMax[i], bestRMin = rBMin[i], bestRMax = rBMax[i];
							bestNL = NL[i], bestNR = NR[i]; // for unsplitting
							bestLMax[a] = bestRMin[a]; // accurate
						}
					}
				}
			}
			// evaluate best split cost
			float noSplitCost = (float)node.triCount * C_INT;
			if (splitCost >= noSplitCost)
			{
				bvhvec3 nodeMin( BVH_FAR ), nodeMax( -BVH_FAR );
				for (uint i = 0; i < node.triCount; i++)
					primIdx[node.leftFirst + i] = fragment[primIdx[node.leftFirst + i]].primIdx;
				break; // not splitting is better.
			}
			// double-buffered partition
			uint A = sliceStart, B = sliceEnd, src = node.leftFirst;
			if (spatial)
			{
				// spatial partitioning
				const float planeDist = (node.aabbMax[bestAxis] - node.aabbMin[bestAxis]) / (HQBVHBINS * 0.9999f);
				const float rPlaneDist = 1.0f / planeDist, nodeMin = node.aabbMin[bestAxis];
				for (uint i = 0; i < node.triCount; i++)
				{
					const uint fragIdx = triIdxA[src++];
					const uint bin1 = (uint)tinybvh_max( (fragment[fragIdx].bmin[bestAxis] - nodeMin) * rPlaneDist, 0.0f );
					const uint bin2 = (uint)tinybvh_max( (fragment[fragIdx].bmax[bestAxis] - nodeMin) * rPlaneDist, 0.0f );
					if (bin2 <= bestPos) triIdxB[A++] = fragIdx; else if (bin1 > bestPos) triIdxB[--B] = fragIdx; else
					{
					#ifdef SBVH_UNSPLITTING
						// unsplitting: 1. Calculate what happens if we add this primitive entirely to the left side
						if (bestNR > 1)
						{
							bvhvec3 unsplitLMin = tinybvh_min( bestLMin, fragment[fragIdx].bmin );
							bvhvec3 unsplitLMax = tinybvh_max( bestLMax, fragment[fragIdx].bmax );
							float AL = (unsplitLMax - unsplitLMin).halfArea();
							float AR = (bestRMax - bestRMin).halfArea();
							float CunsplitLeft = C_TRAV + C_INT * rSAV * (AL * bestNL + AR * (bestNR - 1));
							if (CunsplitLeft < splitCost)
							{
								bestNR--, splitCost = CunsplitLeft, triIdxB[A++] = fragIdx;
								bestLMin = unsplitLMin, bestLMax = unsplitLMax;
								continue;
							}
						}
						// 2. Calculate what happens if we add this primitive entirely to the right side
						if (bestNL > 1)
						{
							const bvhvec3 unsplitRMin = tinybvh_min( bestRMin, fragment[fragIdx].bmin );
							const bvhvec3 unsplitRMax = tinybvh_max( bestRMax, fragment[fragIdx].bmax );
							const float AL = (bestLMax - bestLMin).halfArea();
							const float AR = (unsplitRMax - unsplitRMin).halfArea();
							const float CunsplitRight = C_TRAV + C_INT * rSAV * (AL * (bestNL - 1) + AR * bestNR);
							if (CunsplitRight < splitCost)
							{
								bestNL--, splitCost = CunsplitRight, triIdxB[--B] = fragIdx;
								bestRMin = unsplitRMin, bestRMax = unsplitRMax;
								continue;
							}
						}
					#endif
						// split straddler
						ALIGNED( 64 ) Fragment part1, part2; // keep all clipping in a single cacheline.
						bool leftOK = false, rightOK = false;
						float splitPos = bestLMax[bestAxis];
						SplitFrag( fragment[fragIdx], part1, part2, minDim, bestAxis, splitPos, leftOK, rightOK );
						if (leftOK && rightOK)
							fragment[fragIdx] = part1, triIdxB[A++] = fragIdx,
							fragment[nextFrag] = part2, triIdxB[--B] = nextFrag++;
						else // didn't work out; unsplit (rare)
							if (leftOK) triIdxB[A++] = fragIdx; else triIdxB[--B] = fragIdx;
					}
				}
				// for spatial splits, we fully refresh the bounds: clipping is never fully stable..
				bestLMin = bestRMin = bvhvec3( BVH_FAR ), bestLMax = bestRMax = bvhvec3( -BVH_FAR );
				for (uint i = sliceStart; i < A; i++)
					bestLMin = tinybvh_min( bestLMin, fragment[triIdxB[i]].bmin ),
					bestLMax = tinybvh_max( bestLMax, fragment[triIdxB[i]].bmax );
				for (uint i = B; i < sliceEnd; i++)
					bestRMin = tinybvh_min( bestRMin, fragment[triIdxB[i]].bmin ),
					bestRMax = tinybvh_max( bestRMax, fragment[triIdxB[i]].bmax );
			}
			else
			{
				// object partitioning
				const float rpd = rpd3.cell[bestAxis], nmin = nmin3.cell[bestAxis];
				for (uint i = 0; i < node.triCount; i++)
				{
					const uint fr = primIdx[src + i];
					int32_t bi = (int32_t)(((fragment[fr].bmin[bestAxis] + fragment[fr].bmax[bestAxis]) * 0.5f - nmin) * rpd);
					bi = tinybvh_clamp( bi, 0, HQBVHBINS - 1 );
					if (bi <= (int32_t)bestPos) triIdxB[A++] = fr; else triIdxB[--B] = fr;
				}
			}
			// copy back slice data
			memcpy( triIdxA + sliceStart, triIdxB + sliceStart, (sliceEnd - sliceStart) * 4 );
			// create child nodes
			uint leftCount = A - sliceStart, rightCount = sliceEnd - B;
			if (leftCount == 0 || rightCount == 0) break;
			int32_t leftChildIdx = newNodePtr++, rightChildIdx = newNodePtr++;
			bvhNode[leftChildIdx].aabbMin = bestLMin, bvhNode[leftChildIdx].aabbMax = bestLMax;
			bvhNode[leftChildIdx].leftFirst = sliceStart, bvhNode[leftChildIdx].triCount = leftCount;
			bvhNode[rightChildIdx].aabbMin = bestRMin, bvhNode[rightChildIdx].aabbMax = bestRMax;
			bvhNode[rightChildIdx].leftFirst = B, bvhNode[rightChildIdx].triCount = rightCount;
			node.leftFirst = leftChildIdx, node.triCount = 0;
			// recurse
			task[taskCount].node = rightChildIdx, task[taskCount].sliceEnd = sliceEnd;
			task[taskCount++].sliceStart = sliceEnd = (A + B) >> 1, nodeIdx = leftChildIdx;
		}
		// fetch subdivision task from stack
		if (taskCount == 0) break; else
			nodeIdx = task[--taskCount].node,
			sliceStart = task[taskCount].sliceStart,
			sliceEnd = task[taskCount].sliceEnd;
	}
	// all done.
	AlignedFree( triIdxB );
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
	refittable = false; // can't refit an SBVH
	may_have_holes = false; // there may be holes in the index list, but not in the node list
	usedNodes = newNodePtr;
	Compact();
}

// Refitting: For animated meshes, where the topology remains intact. This
// includes trees waving in the wind, or subsequent frames for skinned
// animations. Repeated refitting tends to lead to deteriorated BVHs and
// slower ray tracing. Rebuild when this happens.
void BVH::Refit( const uint /* unused */ )
{
	FATAL_ERROR_IF( !refittable, "BVH::Refit( .. ), refitting an SBVH." );
	FATAL_ERROR_IF( bvhNode == 0, "BVH::Refit( .. ), bvhNode == 0." );
	FATAL_ERROR_IF( may_have_holes, "BVH::Refit( .. ), bvh may have holes." );
	for (int32_t i = usedNodes - 1; i >= 0; i--)
	{
		BVHNode& node = bvhNode[i];
		if (node.isLeaf()) // leaf: adjust to current triangle vertex positions
		{
			float4 bmin( BVH_FAR ), bmax( -BVH_FAR );
			if (vertIdx)
			{
				for (uint first = node.leftFirst, j = 0; j < node.triCount; j++)
				{
					const uint vidx = primIdx[first + j] * 3;
					const uint i0 = vertIdx[vidx], i1 = vertIdx[vidx + 1], i2 = vertIdx[vidx + 2];
					const float4 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
					const float4 t1 = tinybvh_min( v0, bmin );
					const float4 t2 = tinybvh_max( v0, bmax );
					const float4 t3 = tinybvh_min( v1, v2 );
					const float4 t4 = tinybvh_max( v1, v2 );
					bmin = tinybvh_min( t1, t3 );
					bmax = tinybvh_max( t2, t4 );
				}
			}
			else
			{
				for (uint first = node.leftFirst, j = 0; j < node.triCount; j++)
				{
					const uint vidx = primIdx[first + j] * 3;
					const float4 v0 = verts[vidx], v1 = verts[vidx + 1], v2 = verts[vidx + 2];
					const float4 t1 = tinybvh_min( v0, bmin );
					const float4 t2 = tinybvh_max( v0, bmax );
					const float4 t3 = tinybvh_min( v1, v2 );
					const float4 t4 = tinybvh_max( v1, v2 );
					bmin = tinybvh_min( t1, t3 );
					bmax = tinybvh_max( t2, t4 );
				}
			}
			node.aabbMin = aabbMin, node.aabbMax = aabbMax;
			continue;
		}
		// interior node: adjust to child bounds
		const BVHNode& left = bvhNode[node.leftFirst], & right = bvhNode[node.leftFirst + 1];
		node.aabbMin = tinybvh_min( left.aabbMin, right.aabbMin );
		node.aabbMax = tinybvh_max( left.aabbMax, right.aabbMax );
	}
	aabbMin = bvhNode[0].aabbMin, aabbMax = bvhNode[0].aabbMax;
}

bool BVH::IntersectSphere( const bvhvec3& pos, const float r ) const
{
	const bvhvec3 bmin = pos - bvhvec3( r ), bmax = pos + bvhvec3( r );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0;
	const float r2 = r * r;
	while (1)
	{
		if (node->isLeaf())
		{
			// check if the leaf aabb overlaps the sphere: https://gamedev.stackexchange.com/a/156877
			float dist2 = 0;
			if (pos.x < bmin.x) dist2 += (bmin.x - pos.x) * (bmin.x - pos.x);
			if (pos.x > bmax.x) dist2 += (pos.x - bmax.x) * (pos.x - bmax.x);
			if (pos.y < bmin.y) dist2 += (bmin.y - pos.y) * (bmin.y - pos.y);
			if (pos.y > bmax.y) dist2 += (pos.y - bmax.y) * (pos.y - bmax.y);
			if (pos.z < bmin.z) dist2 += (bmin.z - pos.z) * (bmin.z - pos.z);
			if (pos.z > bmax.z) dist2 += (pos.z - bmax.z) * (pos.z - bmax.z);
			if (dist2 <= r2) 
			{
				// tri/sphere test: https://gist.github.com/yomotsu/d845f21e2e1eb49f647f#file-gistfile1-js-L223
				for( uint i = 0; i < node->triCount; i++ )
				{
					uint idx = primIdx[node->leftFirst + i];
					bvhvec3 a, b, c;
					if (!vertIdx) idx *= 3, a = verts[idx], b = verts[idx + 1], c = verts[idx + 2]; else
					{
						const uint i0 = vertIdx[idx * 3], i1 = vertIdx[idx * 3 + 1], i2 = vertIdx[idx * 3 + 2];
						a = verts[i0], b = verts[i1], c = verts[i2];
					}
					const bvhvec3 A = a - pos, B = b - pos, C = c - pos;
					const float rr = r * r;
					const bvhvec3 V = tinybvh_cross( B - A, C - A );
					const float d = tinybvh_dot( A, V ), e = tinybvh_dot( V, V );
					if (d * d > rr * e) continue;
					const float aa = tinybvh_dot( A, A ), ab = tinybvh_dot( A, B ), ac = tinybvh_dot( A, C );
					const float bb = tinybvh_dot( B, B ), bc = tinybvh_dot( B, C ), cc = tinybvh_dot( C, C );
					if ((aa > rr && ab > aa && ac > aa) ||
						(bb > rr && ab > bb && bc > bb) ||
						(cc > rr && ac > cc && bc > cc)) continue;
					const bvhvec3 AB = B - A, BC = C - B, CA = A - C;
					const float d1 = ab - aa, d2 = bc - bb, d3 = ac - cc;
					const float e1 = tinybvh_dot( AB, AB ), e2 = tinybvh_dot( BC, BC ), e3 = tinybvh_dot( CA, CA );
					const bvhvec3 Q1 = A * e1 - AB * d1, Q2 = B * e2 - BC * d2, Q3 = C * e3 - CA * d3;
					const bvhvec3 QC = C * e1 - Q1, QA = A * e2 - Q2, QB = B * e3 - Q3;
					if ((tinybvh_dot( Q1, Q1 ) > rr * e1 * e1 && tinybvh_dot( Q1, QC ) >= 0) ||
						(tinybvh_dot( Q2, Q2 ) > rr * e2 * e2 && tinybvh_dot( Q2, QA ) >= 0) ||
						(tinybvh_dot( Q3, Q3 ) > rr * e3 * e3 && tinybvh_dot( Q3, QB ) >= 0)) continue;
					// const float dist = sqrtf( d * d / e ) - r; // we're not using this.
					return true; 
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		BVHNode* child1 = &bvhNode[node->leftFirst], * child2 = &bvhNode[node->leftFirst + 1];
		bool hit1 = child1->Intersect( bmin, bmax ), hit2 = child2->Intersect( bmin, bmax );
		if (hit1 && hit2) stack[stackPtr++] = child2, node = child1;
		else if (hit1) node = child1; else if (hit2) node = child2;
		else { if (stackPtr == 0) break; node = stack[--stackPtr]; }
	}
	return false;
}

int32_t BVH::Intersect( Ray& ray ) const
{
	if (isTLAS()) return IntersectTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			// Performance note: if indexed primitives (ENABLE_INDEXED_GEOMETRY) and custom
			// geometry (ENABLE_CUSTOM_GEOMETRY) are both disabled, this leaf code reduces
			// to a regular loop over triangles. Otherwise, the extra flexibility comes at
			// a small performance cost.
			if (indexedEnabled && vertIdx != 0) for (uint i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTriIndexed( ray, verts, vertIdx, primIdx[node->leftFirst + i] );
			else if (customEnabled && customIntersect != 0) for (uint i = 0; i < node->triCount; i++, cost += C_INT)
			{
				if ((*customIntersect)(ray, primIdx[node->leftFirst + i]))
				{
				#if INST_IDX_BITS == 32
					ray.hit.inst = ray.instIdx;
				#else
					ray.hit.prim = (ray.hit.prim & PRIM_IDX_MASK) + ray.instIdx;
				#endif
				}
			}
			else for (uint i = 0; i < node->triCount; i++, cost += C_INT)
				IntersectTri( ray, verts, primIdx[node->leftFirst + i] );
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return cost;
}

int32_t BVH::IntersectTLAS( Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0, cost = 0;
	while (1)
	{
		cost += C_TRAV;
		if (node->isLeaf())
		{
			Ray tmp;
			for (uint i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				const uint instIdx = primIdx[node->leftFirst + i];
				const BLASInstance& inst = instList[instIdx];
				const BVHBase* blas = blasList[inst.blasIdx];
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.instIdx = instIdx << (32 - INST_IDX_BITS);
				tmp.hit = ray.hit;
				// 2. Traverse BLAS with the transformed ray
				// Note: Valid BVH layout options for BLASses are the regular BVH layout,
				// the AVX-optimized BVH_SOA layout and the wide BVH4_CPU layout. If all
				// BLASses are of the same layout this reduces to nearly zero cost for
				// a small set of predictable branches.
				assert( blas->layout == LAYOUT_BVH || blas->layout == LAYOUT_BVH4_CPU || blas->layout == LAYOUT_BVH_SOA );
				if (blas->layout == LAYOUT_BVH)
				{
					// regular (triangle) BVH traversal
					tmp.rD = tinybvh_safercp( tmp.D );
					cost += ((BVH*)blas)->Intersect( tmp );
				}
				else
				{
					tmp.rD = tinybvh_safercp( tmp.D );
					if (blas->layout == LAYOUT_BVH4_CPU) cost += ((BVH4_CPU*)blas)->Intersect( tmp );
					else if (blas->layout == LAYOUT_BVH_SOA) cost += ((BVH_SoA*)blas)->Intersect( tmp );
				}
				// 3. Restore ray
				ray.hit = tmp.hit;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return cost;
}

bool BVH::IsOccluded( const Ray& ray ) const
{
	if (isTLAS()) return IsOccludedTLAS( ray );
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			if (indexedEnabled && vertIdx != 0)
			{
				for (uint i = 0; i < node->triCount; i++)
					if (IndexedTriOccludes( ray, verts, vertIdx, primIdx[node->leftFirst + i] )) return true;
			}
			else if (customEnabled && customIsOccluded != 0)
			{
				for (uint i = 0; i < node->triCount; i++)
					if ((*customIsOccluded)(ray, primIdx[node->leftFirst + i])) return true;
			}
			else
			{
				for (uint i = 0; i < node->triCount; i++)
					if (TriOccludes( ray, verts, primIdx[node->leftFirst + i] )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

bool BVH::IsOccludedTLAS( const Ray& ray ) const
{
	BVHNode* node = &bvhNode[0], * stack[64];
	uint stackPtr = 0;
	Ray tmp;
	tmp.hit = ray.hit;
	while (1)
	{
		if (node->isLeaf())
		{
			for (uint i = 0; i < node->triCount; i++)
			{
				// BLAS traversal
				BLASInstance& inst = instList[primIdx[node->leftFirst + i]];
				BVH* blas = (BVH*)blasList[inst.blasIdx]; // TODO: actually we don't know BVH type.
				// 1. Transform ray with the inverse of the instance transform
				tmp.O = tinybvh_transform_point( ray.O, inst.invTransform );
				tmp.D = tinybvh_transform_vector( ray.D, inst.invTransform );
				tmp.rD = tinybvh_safercp( tmp.D );
				// 2. Traverse BLAS with the transformed ray
				if (blas->IsOccluded( tmp )) return true;
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		BVHNode* child1 = &bvhNode[node->leftFirst];
		BVHNode* child2 = &bvhNode[node->leftFirst + 1];
		float dist1 = child1->Intersect( ray ), dist2 = child2->Intersect( ray );
		if (dist1 > dist2) { tinybvh_swap( dist1, dist2 ); tinybvh_swap( child1, child2 ); }
		if (dist1 == BVH_FAR /* missed both child nodes */)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else /* hit at least one node */
		{
			node = child1; /* continue with the nearest */
			if (dist2 != BVH_FAR) stack[stackPtr++] = child2; /* push far child */
		}
	}
	return false;
}

// Intersect a WALD_32BYTE BVH with a ray packet.
// The 256 rays travel together to better utilize the caches and to amortize the cost
// of memory transfers over the rays in the bundle.
// Note that this basic implementation assumes a specific layout of the rays. Provided
// as 'proof of concept', should not be used in production code.
// Based on Large Ray Packets for Real-time Whitted Ray Tracing, Overbeck et al., 2008,
// extended with sorted traversal and reduced stack traffic.
void BVH::Intersect256Rays( Ray* packet ) const
{
	// convenience macro
#define CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( r ) const bvhvec3 rD = packet[r].rD, t1 = o1 * rD, t2 = o2 * rD; \
	const float tmin = tinybvh_max( tinybvh_max( tinybvh_min( t1.x, t2.x ), tinybvh_min( t1.y, t2.y ) ), tinybvh_min( t1.z, t2.z ) ); \
	const float tmax = tinybvh_min( tinybvh_min( tinybvh_max( t1.x, t2.x ), tinybvh_max( t1.y, t2.y ) ), tinybvh_max( t1.z, t2.z ) );
	// Corner rays are: 0, 51, 204 and 255
	// Construct the bounding planes, with normals pointing outwards
	const bvhvec3 O = packet[0].O; // same for all rays in this case
	const bvhvec3 p0 = packet[0].O + packet[0].D; // top-left
	const bvhvec3 p1 = packet[51].O + packet[51].D; // top-right
	const bvhvec3 p2 = packet[204].O + packet[204].D; // bottom-left
	const bvhvec3 p3 = packet[255].O + packet[255].D; // bottom-right
	const bvhvec3 plane0 = tinybvh_normalize( tinybvh_cross( p0 - O, p0 - p2 ) ); // left plane
	const bvhvec3 plane1 = tinybvh_normalize( tinybvh_cross( p3 - O, p3 - p1 ) ); // right plane
	const bvhvec3 plane2 = tinybvh_normalize( tinybvh_cross( p1 - O, p1 - p0 ) ); // top plane
	const bvhvec3 plane3 = tinybvh_normalize( tinybvh_cross( p2 - O, p2 - p3 ) ); // bottom plane
	const int32_t sign0x = plane0.x < 0 ? 4 : 0, sign0y = plane0.y < 0 ? 5 : 1, sign0z = plane0.z < 0 ? 6 : 2;
	const int32_t sign1x = plane1.x < 0 ? 4 : 0, sign1y = plane1.y < 0 ? 5 : 1, sign1z = plane1.z < 0 ? 6 : 2;
	const int32_t sign2x = plane2.x < 0 ? 4 : 0, sign2y = plane2.y < 0 ? 5 : 1, sign2z = plane2.z < 0 ? 6 : 2;
	const int32_t sign3x = plane3.x < 0 ? 4 : 0, sign3y = plane3.y < 0 ? 5 : 1, sign3z = plane3.z < 0 ? 6 : 2;
	const float d0 = tinybvh_dot( O, plane0 ), d1 = tinybvh_dot( O, plane1 );
	const float d2 = tinybvh_dot( O, plane2 ), d3 = tinybvh_dot( O, plane3 );
	// Traverse the tree with the packet
	int32_t first = 0, last = 255; // first and last active ray in the packet
	const BVHNode* node = &bvhNode[0];
	ALIGNED( 64 ) uint stack[64], stackPtr = 0;
	while (1)
	{
		if (node->isLeaf())
		{
			// handle leaf node
			for (uint j = 0; j < node->triCount; j++)
			{
				const uint idx = primIdx[node->leftFirst + j], vid = idx * 3;
				const bvhvec3 edge1 = verts[vid + 1] - verts[vid], edge2 = verts[vid + 2] - verts[vid];
				const bvhvec3 s = O - bvhvec3( verts[vid] );
				for (int32_t i = first; i <= last; i++)
				{
					Ray& ray = packet[i];
					const bvhvec3 h = tinybvh_cross( ray.D, edge2 );
					const float a = tinybvh_dot( edge1, h );
					if (fabs( a ) < 0.0000001f) continue; // ray parallel to triangle
					const float f = 1 / a, u = f * tinybvh_dot( s, h );
					if (u < 0 || u > 1) continue;
					const bvhvec3 q = tinybvh_cross( s, edge1 );
					const float v = f * tinybvh_dot( ray.D, q );
					if (v < 0 || u + v > 1) continue;
					const float t = f * tinybvh_dot( edge2, q );
					if (t <= 0 || t >= ray.hit.t) continue;
					ray.hit.t = t, ray.hit.u = u, ray.hit.v = v;
				#if INST_IDX_BITS == 32
					ray.hit.prim = idx;
					ray.hit.inst = ray.instIdx;
				#else
					ray.hit.prim = idx + ray.instIdx;
				#endif
				}
			}
			if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
		else
		{
			// fetch pointers to child nodes
			const BVHNode* left = bvhNode + node->leftFirst;
			const BVHNode* right = bvhNode + node->leftFirst + 1;
			bool visitLeft = true, visitRight = true;
			int32_t leftFirst = first, leftLast = last, rightFirst = first, rightLast = last;
			float distLeft, distRight;
			{
				// see if we want to intersect the left child
				const bvhvec3 o1( left->aabbMin.x - O.x, left->aabbMin.y - O.y, left->aabbMin.z - O.z );
				const bvhvec3 o2( left->aabbMax.x - O.x, left->aabbMax.y - O.y, left->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distLeft = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					float* minmax = (float*)left;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > d0 || tinybvh_dot( c1, plane1 ) > d1 ||
						tinybvh_dot( c2, plane2 ) > d2 || tinybvh_dot( c3, plane3 ) > d3)
						visitLeft = false;
					else // 3. Last resort: update first and last, stay in node if first > last
					{
						for (; leftFirst <= leftLast; leftFirst++)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( leftFirst );
							if (tmax >= tmin && tmin < packet[leftFirst].hit.t && tmax >= 0) { distLeft = tmin; break; }
						}
						for (; leftLast >= leftFirst; leftLast--)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( leftLast );
							if (tmax >= tmin && tmin < packet[leftLast].hit.t && tmax >= 0) break;
						}
						visitLeft = leftLast >= leftFirst;
					}
				}
			}
			{
				// see if we want to intersect the right child
				const bvhvec3 o1( right->aabbMin.x - O.x, right->aabbMin.y - O.y, right->aabbMin.z - O.z );
				const bvhvec3 o2( right->aabbMax.x - O.x, right->aabbMax.y - O.y, right->aabbMax.z - O.z );
				// 1. Early-in test: if first ray hits the node, the packet visits the node
				bool earlyHit;
				{
					CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( first );
					earlyHit = (tmax >= tmin && tmin < packet[first].hit.t && tmax >= 0);
					distRight = tmin;
				}
				if (!earlyHit) // 2. Early-out test: if the node aabb is outside the four planes, we skip the node
				{
					float* minmax = (float*)right;
					bvhvec3 c0( minmax[sign0x], minmax[sign0y], minmax[sign0z] );
					bvhvec3 c1( minmax[sign1x], minmax[sign1y], minmax[sign1z] );
					bvhvec3 c2( minmax[sign2x], minmax[sign2y], minmax[sign2z] );
					bvhvec3 c3( minmax[sign3x], minmax[sign3y], minmax[sign3z] );
					if (tinybvh_dot( c0, plane0 ) > d0 || tinybvh_dot( c1, plane1 ) > d1 ||
						tinybvh_dot( c2, plane2 ) > d2 || tinybvh_dot( c3, plane3 ) > d3)
						visitRight = false;
					else // 3. Last resort: update first and last, stay in node if first > last
					{
						for (; rightFirst <= rightLast; rightFirst++)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( rightFirst );
							if (tmax >= tmin && tmin < packet[rightFirst].hit.t && tmax >= 0) { distRight = tmin; break; }
						}
						for (; rightLast >= first; rightLast--)
						{
							CALC_TMIN_TMAX_WITH_SLABTEST_ON_RAY( rightLast );
							if (tmax >= tmin && tmin < packet[rightLast].hit.t && tmax >= 0) break;
						}
						visitRight = rightLast >= rightFirst;
					}
				}
			}
			// process intersection result
			if (visitLeft && visitRight)
			{
				if (distLeft < distRight) // push right, continue with left
				{
					stack[stackPtr++] = node->leftFirst + 1;
					stack[stackPtr++] = (rightFirst << 8) + rightLast;
					node = left, first = leftFirst, last = leftLast;
				}
				else // push left, continue with right
				{
					stack[stackPtr++] = node->leftFirst;
					stack[stackPtr++] = (leftFirst << 8) + leftLast;
					node = right, first = rightFirst, last = rightLast;
				}
			}
			else if (visitLeft) // continue with left
				node = left, first = leftFirst, last = leftLast;
			else if (visitRight) // continue with right
				node = right, first = rightFirst, last = rightLast;
			else if (stackPtr == 0) break; else // pop
				last = stack[--stackPtr], node = bvhNode + stack[--stackPtr],
				first = last >> 8, last &= 255;
		}
	}
}

int32_t BVH::NodeCount() const
{
	// Determine the number of nodes in the tree. Typically the result should
	// be usedNodes - 1 (second node is always unused), but some builders may
	// have unused nodes besides node 1. TODO: Support more layouts.
	uint retVal = 0, nodeIdx = 0, stack[64], stackPtr = 0;
	while (1)
	{
		const BVHNode& n = bvhNode[nodeIdx];
		retVal++;
		if (n.isLeaf()) { if (stackPtr == 0) break; else nodeIdx = stack[--stackPtr]; }
		else nodeIdx = n.leftFirst, stack[stackPtr++] = n.leftFirst + 1;
	}
	return retVal;
}

// Compact: Reduce the size of a BVH by removing any unsed nodes.
// This is useful after an SBVH build or multi-threaded build, but also after
// calling MergeLeafs. Some operations, such as Optimize, *require* a
// compacted tree to work correctly.
void BVH::Compact()
{
	FATAL_ERROR_IF( bvhNode == 0, "BVH::Compact(), bvhNode == 0." );
	BVHNode* tmp = (BVHNode*)AlignedAlloc( sizeof( BVHNode ) * allocatedNodes /* do *not* trim */ );
	memcpy( tmp, bvhNode, 2 * sizeof( BVHNode ) );
	newNodePtr = 2;
	uint nodeIdx = 0, stack[128], stackPtr = 0;
	while (1)
	{
		BVHNode& node = tmp[nodeIdx];
		const BVHNode& left = bvhNode[node.leftFirst];
		const BVHNode& right = bvhNode[node.leftFirst + 1];
		tmp[newNodePtr] = left, tmp[newNodePtr + 1] = right;
		const uint todo1 = newNodePtr, todo2 = newNodePtr + 1;
		node.leftFirst = newNodePtr, newNodePtr += 2;
		if (!left.isLeaf()) stack[stackPtr++] = todo1;
		if (!right.isLeaf()) stack[stackPtr++] = todo2;
		if (!stackPtr) break;
		nodeIdx = stack[--stackPtr];
	}
	usedNodes = newNodePtr;
	AlignedFree( bvhNode );
	bvhNode = tmp;
}