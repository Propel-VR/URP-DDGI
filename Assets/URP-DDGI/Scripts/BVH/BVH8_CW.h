struct ALIGNED( 32 ) bvhaabb
{
	bvhvec3 minBounds; uint32_t dummy1;
	bvhvec3 maxBounds; uint32_t dummy2;
};

struct bvhvec4slice
{
	bvhvec4slice() = default;
	bvhvec4slice( const bvhvec4* data, uint32_t count, uint32_t stride = sizeof( bvhvec4 ) );
	operator bool() const { return !!data; }
	const bvhvec4& operator [] ( size_t i ) const;
	const int8_t* data = nullptr;
	uint32_t count, stride;
};

class BVH8_CWBVH : public BVHBase
{
public:
	BVH8_CWBVH( BVHContext ctx = {} ) { layout = LAYOUT_CWBVH; context = ctx; }
	BVH8_CWBVH( MBVH<8>& bvh8 ) { /* DEPRECATED */ layout = LAYOUT_CWBVH; ConvertFrom( bvh8 ); }
	~BVH8_CWBVH();
	void Save( const char* fileName );
	bool Load( const char* fileName, const uint32_t expectedTris );
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void ConvertFrom( MBVH<8>& original, bool compact = true );
	int32_t Intersect( Ray& ray ) const;
	bool IsOccluded( const Ray& ray ) const { FALLBACK_SHADOW_QUERY( ray ); }
	// BVH8 data
	bvhvec4* bvh8Data = 0;			// nodes in CWBVH format.
	bvhvec4* bvh8Tris = 0;			// triangle data for CWBVH nodes.
	uint32_t allocatedBlocks = 0;	// node data is stored in blocks of 16 byte.
	uint32_t usedBlocks = 0;		// actually used blocks.
	MBVH<8> bvh8;					// BVH8_CWBVH is created from BVH8 and uses its data.
	bool ownBVH8 = true;			// false when ConvertFrom receives an external bvh8.
};

// BLASInstance: A TLAS is built over BLAS instances, where a single BLAS can be
// used with multiple transforms, and multiple BLASses can be combined in a complex
// scene. The TLAS is built over the world-space AABBs of the BLAS root nodes.
class ALIGNED( 64 ) BLASInstance
{
public:
	BLASInstance() = default;
	BLASInstance( uint32_t idx ) : blasIdx( idx ) {}
	float transform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	float invTransform[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; // identity
	bvhvec3 aabbMin = bvhvec3( BVH_FAR );
	uint32_t blasIdx = 0;
	bvhvec3 aabbMax = bvhvec3( -BVH_FAR );
	uint32_t dummy[9]; // pad struct to 64 byte
	void Update( BVHBase * blas );
	void InvertTransform();
};







BVH8_CWBVH::~BVH8_CWBVH()
{
	if (!ownBVH8) bvh8 = MBVH<8>(); // clear out pointers we don't own.
	AlignedFree( bvh8Data );
	AlignedFree( bvh8Tris );
}

void BVH8_CWBVH::Save( const char* fileName )
{
	std::fstream s{ fileName, s.binary | s.out };
	uint32_t header = TINY_BVH_VERSION_SUB + (TINY_BVH_VERSION_MINOR << 8) + (TINY_BVH_VERSION_MAJOR << 16) + (layout << 24);
	s.write( (char*)&header, sizeof( uint32_t ) );
	s.write( (char*)&triCount, sizeof( uint32_t ) );
	s.write( (char*)this, sizeof( BVH8_CWBVH ) );
	s.write( (char*)bvh8Data, usedBlocks * 16 );
	s.write( (char*)bvh8Tris, bvh8.idxCount * 4 * 16 );
}

bool BVH8_CWBVH::Load( const char* fileName, const uint32_t expectedTris )
{
	// open file and check contents
	std::fstream s{ fileName, s.binary | s.in };
	if (!s) return false;
	BVHContext tmp = context;
	uint32_t header, fileTriCount;
	s.read( (char*)&header, sizeof( uint32_t ) );
	if (((header >> 8) & 255) != TINY_BVH_VERSION_MINOR ||
		((header >> 16) & 255) != TINY_BVH_VERSION_MAJOR ||
		(header & 255) != TINY_BVH_VERSION_SUB || (header >> 24) != layout) return false;
	s.read( (char*)&fileTriCount, sizeof( uint32_t ) );
	if (fileTriCount != expectedTris) return false;
	// all checks passed; safe to overwrite *this
	s.read( (char*)this, sizeof( BVH8_CWBVH ) );
	context = tmp; // can't load context; function pointers will differ.
	bvh8Data = (bvhvec4*)AlignedAlloc( usedBlocks * 16 );
	bvh8Tris = (bvhvec4*)AlignedAlloc( bvh8.idxCount * 4 * 16 );
	allocatedBlocks = usedBlocks;
	s.read( (char*)bvh8Data, usedBlocks * 16 );
	s.read( (char*)bvh8Tris, bvh8.idxCount * 4 * 16 );
	bvh8 = MBVH<8>();
	return true;
}

void BVH8_CWBVH::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH8_CWBVH::Build( const bvhvec4slice& vertices )
{
	bvh8.context = context; // properly propagate context to fix issue #66.
	bvh8.Build( vertices );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH8_CWBVH::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh8.context = context;
	bvh8.Build( vertices, indices, prims );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4slice& vertices )
{
	bvh8.context = context; // properly propagate context to fix issue #66.
	bvh8.BuildHQ( vertices );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

void BVH8_CWBVH::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh8.context = context;
	bvh8.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh8, true );
}

void BVH8_CWBVH::ConvertFrom( MBVH<8>& original, bool )
{
	// get a copy of the original bvh8
	if (&original != &bvh8) ownBVH8 = false; // bvh isn't ours; don't delete in destructor.
	bvh8 = original;
	// Convert a BVH8 to the format specified in: "Efficient Incoherent Ray
	// Traversal on GPUs Through Compressed Wide BVHs", Ylitie et al. 2017.
	// Adapted from code by "AlanWBFT".
	FATAL_ERROR_IF( bvh8.mbvhNode[0].isLeaf(), "BVH8_CWBVH::ConvertFrom( .. ), converting a single-node bvh." );
	// allocate memory
	// Note: This can be far lower (specifically: usedNodes) if we know that
	// none of the BVH8 leafs has more than three primitives.
	// Without this guarantee, the only safe upper limit is triCount * 2, since
	// we will be splitting fat BVH8 leafs to as we go.
	uint32_t spaceNeeded = bvh8.triCount * 2 * 5; // CWBVH nodes use 80 bytes each.
	if (spaceNeeded > allocatedBlocks)
	{
		bvh8Data = (bvhvec4*)AlignedAlloc( spaceNeeded * 16 );
		bvh8Tris = (bvhvec4*)AlignedAlloc( bvh8.idxCount * 4 * 16 );
		allocatedBlocks = spaceNeeded;
	}
	memset( bvh8Data, 0, spaceNeeded * 16 );
	memset( bvh8Tris, 0, bvh8.idxCount * 3 * 16 );
	CopyBasePropertiesFrom( bvh8 );
	MBVH<8>::MBVHNode* stackNodePtr[256];
	uint32_t stackNodeAddr[256], stackPtr = 1, nodeDataPtr = 5, triDataPtr = 0;
	stackNodePtr[0] = &bvh8.mbvhNode[0], stackNodeAddr[0] = 0;
	// start conversion
	while (stackPtr > 0)
	{
		MBVH<8>::MBVHNode* orig = stackNodePtr[--stackPtr];
		const int32_t currentNodeAddr = stackNodeAddr[stackPtr];
		bvhvec3 nodeLo = orig->aabbMin, nodeHi = orig->aabbMax;
		// greedy child node ordering
		const bvhvec3 nodeCentroid = (nodeLo + nodeHi) * 0.5f;
		float cost[8][8];
		int32_t assignment[8];
		bool isSlotEmpty[8];
		for (int32_t s = 0; s < 8; s++)
		{
			isSlotEmpty[s] = true, assignment[s] = -1;
			bvhvec3 ds(
				(((s >> 2) & 1) == 1) ? -1.0f : 1.0f,
				(((s >> 1) & 1) == 1) ? -1.0f : 1.0f,
				(((s >> 0) & 1) == 1) ? -1.0f : 1.0f
			);
			for (int32_t i = 0; i < 8; i++) if (orig->child[i] == 0) cost[s][i] = BVH_FAR; else
			{
				MBVH<8>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
				if (child->triCount > 3 /* must be leaf */) bvh8.SplitBVHLeaf( orig->child[i], 3 );
				bvhvec3 childCentroid = (child->aabbMin + child->aabbMax) * 0.5f;
				cost[s][i] = tinybvh_dot( childCentroid - nodeCentroid, ds );
			}
		}
		while (1)
		{
			float minCost = BVH_FAR;
			int32_t minEntryx = -1, minEntryy = -1;
			for (int32_t s = 0; s < 8; s++) for (int32_t i = 0; i < 8; i++)
				if (assignment[i] == -1 && isSlotEmpty[s] && cost[s][i] < minCost)
					minCost = cost[s][i], minEntryx = s, minEntryy = i;
			if (minEntryx == -1 && minEntryy == -1) break;
			isSlotEmpty[minEntryx] = false, assignment[minEntryy] = minEntryx;
		}
		for (int32_t i = 0; i < 8; i++) if (assignment[i] == -1) for (int32_t s = 0; s < 8; s++) if (isSlotEmpty[s])
		{
			isSlotEmpty[s] = false, assignment[i] = s;
			break;
		}
		const MBVH<8>::MBVHNode oldNode = *orig;
		for (int32_t i = 0; i < 8; i++) orig->child[assignment[i]] = oldNode.child[i];
		// calculate quantization parameters for each axis
		const int32_t ex = (int32_t)((int8_t)ceilf( log2f( (nodeHi.x - nodeLo.x) / 255.0f ) ));
		const int32_t ey = (int32_t)((int8_t)ceilf( log2f( (nodeHi.y - nodeLo.y) / 255.0f ) ));
		const int32_t ez = (int32_t)((int8_t)ceilf( log2f( (nodeHi.z - nodeLo.z) / 255.0f ) ));
		// encode output
		int32_t internalChildCount = 0, leafChildTriCount = 0, childBaseIndex = 0, triangleBaseIndex = 0;
		uint8_t imask = 0;
		for (int32_t i = 0; i < 8; i++)
		{
			if (orig->child[i] == 0) continue;
			MBVH<8>::MBVHNode* const child = &bvh8.mbvhNode[orig->child[i]];
			const int32_t qlox = (int32_t)floorf( (child->aabbMin.x - nodeLo.x) / powf( 2, (float)ex ) );
			const int32_t qloy = (int32_t)floorf( (child->aabbMin.y - nodeLo.y) / powf( 2, (float)ey ) );
			const int32_t qloz = (int32_t)floorf( (child->aabbMin.z - nodeLo.z) / powf( 2, (float)ez ) );
			const int32_t qhix = (int32_t)ceilf( (child->aabbMax.x - nodeLo.x) / powf( 2, (float)ex ) );
			const int32_t qhiy = (int32_t)ceilf( (child->aabbMax.y - nodeLo.y) / powf( 2, (float)ey ) );
			const int32_t qhiz = (int32_t)ceilf( (child->aabbMax.z - nodeLo.z) / powf( 2, (float)ez ) );
			uint8_t* const baseAddr = (uint8_t*)&bvh8Data[currentNodeAddr + 2];
			baseAddr[i + 0] = (uint8_t)qlox, baseAddr[i + 24] = (uint8_t)qhix;
			baseAddr[i + 8] = (uint8_t)qloy, baseAddr[i + 32] = (uint8_t)qhiy;
			baseAddr[i + 16] = (uint8_t)qloz, baseAddr[i + 40] = (uint8_t)qhiz;
			if (!child->isLeaf())
			{
				// interior node, set params and push onto stack
				const int32_t childNodeAddr = nodeDataPtr;
				if (internalChildCount++ == 0) childBaseIndex = childNodeAddr / 5;
				nodeDataPtr += 5, imask |= 1 << i;
				// set the meta field - This calculation assumes children are stored contiguously.
				uint8_t* const childMetaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
				childMetaField[i] = (1 << 5) | (24 + (uint8_t)i); // I don't see how this accounts for empty children?
				stackNodePtr[stackPtr] = child, stackNodeAddr[stackPtr++] = childNodeAddr; // counted in float4s
				internalChildCount++;
				continue;
			}
			// leaf node
			const uint32_t tcount = tinybvh_min( child->triCount, 3u ); // TODO: ensure that's the case; clamping for now.
			if (leafChildTriCount == 0) triangleBaseIndex = triDataPtr;
			int32_t unaryEncodedTriCount = tcount == 1 ? 0b001 : tcount == 2 ? 0b011 : 0b111;
			// set the meta field - This calculation assumes children are stored contiguously.
			uint8_t* const childMetaField = ((uint8_t*)&bvh8Data[currentNodeAddr + 1]) + 8;
			childMetaField[i] = (uint8_t)((unaryEncodedTriCount << 5) | leafChildTriCount);
			leafChildTriCount += tcount;
			for (uint32_t j = 0; j < tcount; j++)
			{
				int32_t triIdx = bvh8.bvh.primIdx[child->firstTri + j];
				uint32_t ti0, ti1, ti2;
				if (bvh8.bvh.vertIdx)
					ti0 = bvh8.bvh.vertIdx[triIdx * 3], 
					ti1 = bvh8.bvh.vertIdx[triIdx * 3 + 1],
					ti2 = bvh8.bvh.vertIdx[triIdx * 3 + 2];
				else
					ti0 = triIdx * 3, ti1 = triIdx * 3 + 1, ti2 = triIdx * 3 + 2;
			#ifdef CWBVH_COMPRESSED_TRIS
				PrecomputeTriangle( verts, ti0, ti1, ti2, (float*)&bvh8Tris[triDataPtr] );
				bvh8Tris[triDataPtr + 3] = bvhvec4( 0, 0, 0, *(float*)&triIdx );
				triDataPtr += 4;
			#else
				bvhvec4 t = bvh8.bvh.verts[ti0];
				bvh8Tris[triDataPtr + 0] = bvh8.bvh.verts[ti2] - t;
				bvh8Tris[triDataPtr + 1] = bvh8.bvh.verts[ti1] - t;
				t.w = *(float*)&triIdx;
				bvh8Tris[triDataPtr + 2] = t, triDataPtr += 3;
			#endif
			}
		}
		uint8_t exyzAndimask[4] = { *(uint8_t*)&ex, *(uint8_t*)&ey, *(uint8_t*)&ez, imask };
		bvh8Data[currentNodeAddr + 0] = bvhvec4( nodeLo, *(float*)&exyzAndimask );
		bvh8Data[currentNodeAddr + 1].x = *(float*)&childBaseIndex;
		bvh8Data[currentNodeAddr + 1].y = *(float*)&triangleBaseIndex;
	}
	usedBlocks = nodeDataPtr;
}