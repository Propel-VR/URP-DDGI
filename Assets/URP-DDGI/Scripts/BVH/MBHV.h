template <int M> class MBVH : public BVHBase
{
public:
	struct MBVHNode
	{
		// 4-wide (aka 'shallow') BVH layout.
		bvhvec3 aabbMin; uint32_t firstTri;
		bvhvec3 aabbMax; uint32_t triCount;
		uint32_t child[M];
		uint32_t childCount;
		uint32_t dummy[((30 - M) & 3) + 1]; // dummies are for alignment.
		bool isLeaf() const { return triCount > 0; }
	};
	MBVH( BVHContext ctx = {} ) { layout = LAYOUT_MBVH; context = ctx; }
	MBVH( const BVH& original ) { /* DEPRECATED */ layout = LAYOUT_MBVH; ConvertFrom( original ); }
	~MBVH();
	void Build( const bvhvec4* vertices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices );
	void Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void Build( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4* vertices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices );
	void BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t primCount );
	void BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, const uint32_t primCount );
	void Refit( const uint32_t nodeIdx = 0 );
	void ConvertFrom( const BVH& original, bool compact = true );
	void SplitBVHLeaf( const uint32_t nodeIdx, const uint32_t maxPrims );
	// BVH data
	MBVHNode* mbvhNode = 0;			// BVH node for M-wide BVH.
	BVH bvh;						// BVH4 is created from BVH and uses its data.
	bool ownBVH = true;				// False when ConvertFrom receives an external bvh.
};



template<int M> MBVH<M>::~MBVH()
{
	if (!ownBVH) bvh = BVH(); // clear out pointers we don't own.
	AlignedFree( mbvhNode );
}

template<int M> void MBVH<M>::Build( const bvhvec4* vertices, const uint32_t primCount )
{
	Build( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

template<int M> void MBVH<M>::Build( const bvhvec4slice& vertices )
{
	bvh.context = context; // properly propagate context to fix issue #66.
	bvh.BuildDefault( vertices );
	ConvertFrom( bvh, false );
}

template<int M> void MBVH<M>::Build( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	// build the BVH with a continuous array of bvhvec4 vertices, indexed by 'indices'.
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

template<int M> void MBVH<M>::Build( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	// build the BVH from vertices stored in a slice, indexed by 'indices'.
	bvh.context = context;
	bvh.BuildDefault( vertices, indices, prims );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4* vertices, const uint32_t primCount )
{
	BuildHQ( bvhvec4slice( vertices, primCount * 3, sizeof( bvhvec4 ) ) );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4slice& vertices )
{
	bvh.context = context;
	bvh.BuildHQ( vertices );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4* vertices, const uint32_t* indices, const uint32_t prims )
{
	Build( bvhvec4slice{ vertices, prims * 3, sizeof( bvhvec4 ) }, indices, prims );
}

template<int M> void MBVH<M>::BuildHQ( const bvhvec4slice& vertices, const uint32_t* indices, uint32_t prims )
{
	bvh.context = context;
	bvh.BuildHQ( vertices, indices, prims );
	ConvertFrom( bvh, true );
}

template<int M> void MBVH<M>::Refit( const uint32_t nodeIdx )
{
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.isLeaf())
	{
		bvhvec3 aabbMin( BVH_FAR ), aabbMax( -BVH_FAR );
		if (bvh.vertIdx)
		{
			for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
			{
				const uint32_t vidx = bvh.primIdx[first + j] * 3;
				const uint32_t i0 = bvh.vertIdx[vidx], i1 = bvh.vertIdx[vidx + 1], i2 = bvh.vertIdx[vidx + 2];
				const bvhvec3 v0 = bvh.verts[i0], v1 = bvh.verts[i1], v2 = bvh.verts[i2];
				aabbMin = tinybvh_min( aabbMin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
				aabbMax = tinybvh_max( aabbMax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
			}
		}
		else
		{
			for (uint32_t first = node.firstTri, j = 0; j < node.triCount; j++)
			{
				const uint32_t vidx = bvh.primIdx[first + j] * 3;
				const bvhvec3 v0 = bvh.verts[vidx], v1 = bvh.verts[vidx + 1], v2 = bvh.verts[vidx + 2];
				aabbMin = tinybvh_min( aabbMin, tinybvh_min( tinybvh_min( v0, v1 ), v2 ) );
				aabbMax = tinybvh_max( aabbMax, tinybvh_max( tinybvh_max( v0, v1 ), v2 ) );
			}
		}
		node.aabbMin = aabbMin, node.aabbMax = aabbMax;
	}
	else
	{
		for (unsigned i = 0; i < node.childCount; i++) Refit( node.child[i] );
		MBVHNode& firstChild = mbvhNode[node.child[0]];
		bvhvec3 bmin = firstChild.aabbMin, bmax = firstChild.aabbMax;
		for (unsigned i = 1; i < node.childCount; i++)
		{
			MBVHNode& child = mbvhNode[node.child[i]];
			bmin = tinybvh_min( bmin, child.aabbMin );
			bmax = tinybvh_max( bmax, child.aabbMax );
		}
	}
	if (nodeIdx == 0) aabbMin = node.aabbMin, aabbMax = node.aabbMax;
}

template<int M> void MBVH<M>::ConvertFrom( const BVH& original, bool compact )
{
	// get a copy of the original bvh
	if (&original != &bvh) ownBVH = false; // bvh isn't ours; don't delete in destructor.
	bvh = original;
	// allocate space
	uint32_t spaceNeeded = compact ? original.usedNodes : original.allocatedNodes;
	constexpr bool M8 = M == 8;
	if (M8) spaceNeeded += original.usedNodes >> 1; // cwbvh / SplitLeafs
	if (allocatedNodes < spaceNeeded)
	{
		AlignedFree( mbvhNode );
		mbvhNode = (MBVHNode*)AlignedAlloc( spaceNeeded * sizeof( MBVHNode ) );
		allocatedNodes = spaceNeeded;
	}
	memset( mbvhNode, 0, sizeof( MBVHNode ) * spaceNeeded );
	CopyBasePropertiesFrom( original );
	// create an mbvh node for each bvh2 node
	for (uint32_t i = 0; i < original.usedNodes; i++) if (i != 1)
	{
		BVH::BVHNode& orig = original.bvhNode[i];
		MBVHNode& node = this->mbvhNode[i];
		node.aabbMin = orig.aabbMin, node.aabbMax = orig.aabbMax;
		if (orig.isLeaf()) node.triCount = orig.triCount, node.firstTri = orig.leftFirst;
		else node.child[0] = orig.leftFirst, node.child[1] = orig.leftFirst + 1, node.childCount = 2;
	}
	// collapse
	uint32_t stack[128], stackPtr = 0, nodeIdx = 0; // i.e., root node
	while (1)
	{
		MBVHNode& node = this->mbvhNode[nodeIdx];
		while (node.childCount < M)
		{
			int32_t bestChild = -1;
			float bestChildSA = 0;
			for (uint32_t i = 0; i < node.childCount; i++)
			{
				// see if we can adopt child i
				const MBVHNode& child = this->mbvhNode[node.child[i]];
				if (!child.isLeaf() && node.childCount - 1 + child.childCount <= M)
				{
					const float childSA = SA( child.aabbMin, child.aabbMax );
					if (childSA > bestChildSA) bestChild = i, bestChildSA = childSA;
				}
			}
			if (bestChild == -1) break; // could not adopt
			const MBVHNode& child = this->mbvhNode[node.child[bestChild]];
			node.child[bestChild] = child.child[0];
			for (uint32_t i = 1; i < child.childCount; i++)
				node.child[node.childCount++] = child.child[i];
		}
		// we're done with the node; proceed with the children.
		for (uint32_t i = 0; i < node.childCount; i++)
		{
			const uint32_t childIdx = node.child[i];
			const MBVHNode& child = this->mbvhNode[childIdx];
			if (!child.isLeaf()) stack[stackPtr++] = childIdx;
		}
		if (stackPtr == 0) break;
		nodeIdx = stack[--stackPtr];
	}
	// special case where root is leaf: add extra level - cwbvh needs this.
	MBVHNode& root = this->mbvhNode[0];
	if (root.isLeaf())
	{
		mbvhNode[1] = root;
		root.childCount = 1;
		root.child[0] = 1;
		root.triCount = 0;
	}
	// finalize
	usedNodes = original.usedNodes;
	this->may_have_holes = true;
}

// SplitBVH8Leaf: CWBVH requires that a leaf has no more than 3 primitives,
// but regular BVH construction does not guarantee this. So, here we split
// busy leafs recursively in multiple leaves, until the requirement is met.
template<int M> void MBVH<M>::SplitBVHLeaf( const uint32_t nodeIdx, const uint32_t maxPrims )
{
	const uint32_t* primIdx = bvh.primIdx;
	const Fragment* fragment = bvh.fragment;
	MBVHNode& node = mbvhNode[nodeIdx];
	if (node.triCount <= maxPrims) return; // also catches interior nodes
	// place all primitives in a new node and make this the first child of 'node'
	MBVHNode& firstChild = mbvhNode[node.child[0] = usedNodes++];
	firstChild.triCount = node.triCount;
	firstChild.firstTri = node.firstTri;
	uint32_t nextChild = 1;
	// share with new sibling nodes
	while (firstChild.triCount > maxPrims && nextChild < M)
	{
		MBVHNode& child = mbvhNode[node.child[nextChild] = usedNodes++];
		firstChild.triCount -= maxPrims, child.triCount = maxPrims;
		child.firstTri = firstChild.firstTri + firstChild.triCount;
		nextChild++;
	}
	for (uint32_t i = 0; i < nextChild; i++)
	{
		MBVHNode& child = mbvhNode[node.child[i]];
		if (!refittable) child.aabbMin = node.aabbMin, child.aabbMax = node.aabbMax; else
		{
			// TODO: why is this producing wrong aabbs for SBVH?
			child.aabbMin = bvhvec3( BVH_FAR ), child.aabbMax = bvhvec3( -BVH_FAR );
			for (uint32_t fi, j = 0; j < child.triCount; j++) fi = primIdx[child.firstTri + j],
				child.aabbMin = tinybvh_min( child.aabbMin, fragment[fi].bmin ),
				child.aabbMax = tinybvh_max( child.aabbMax, fragment[fi].bmax );
		}
	}
	node.triCount = 0;
	// recurse; should be rare
	if (firstChild.triCount > maxPrims) SplitBVHLeaf( node.child[0], maxPrims );
}