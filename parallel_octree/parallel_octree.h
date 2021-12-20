#pragma once

#include <cstdint>
#include <memory>

#include "chunk_allocator.h"
#include "chunk_pool.h"

class parallel_octree final
{
public:
	static constexpr uint32_t InvalidIndex = 0xFFFFFFFFu;

	struct point final
	{
		float X, Y, Z;
	};

	struct aabb final
	{
		point Min, Max;
	};

	struct shape_data final
	{
		aabb AABB;
		uint32_t Index;
	};

	struct shape_move final
	{
		aabb aabbOld, aabbNew;
		uint32_t Index;
	};

private:
	struct node;
	struct tree;
	struct leaf;
	struct leaf_extension;

	template <bool Synchronized>
	class traverser;

	template <bool Synchronized>
	class traverser_common;

	template <bool Synchronized>
	class traverser_add;

	template <bool Synchronized>
	class traverser_remove;

	template <bool Synchronized>
	class traverser_move;

private:
	chunk_allocator<> _chunkAllocator;
	chunk_pool<true> _chunkPool;

	node* _root;
	uint32_t _sizeLog;

public:
	explicit parallel_octree(uint32_t sizeLog, size_t bufferSize);
	~parallel_octree();

	void add_synchronized(const shape_data& shapeData);
	void remove_synchronized(const shape_data& shapeData);
	void move_synchronized(const shape_move& shapeMove);

private:
	template <bool Synchronized>
	node* allocate_node(bool isTree);

	template <typename TNode, bool Synchronized>
	TNode* allocate_node();

	float field_size() const;
	aabb initial_aabb() const;

	static aabb aabb_0(const aabb& aabb, const point& centre);
	static aabb aabb_1(const aabb& aabb, const point& centre);
	static aabb aabb_2(const aabb& aabb, const point& centre);
	static aabb aabb_3(const aabb& aabb, const point& centre);
	static aabb aabb_4(const aabb& aabb, const point& centre);
	static aabb aabb_5(const aabb& aabb, const point& centre);
	static aabb aabb_6(const aabb& aabb, const point& centre);
	static aabb aabb_7(const aabb& aabb, const point& centre);

	static bool are_intersected(const aabb& left, const aabb& right);
	static bool are_intersected(const shape_data& shape, const aabb& aabb);

	static point calculate_centre(const aabb& aabb);
};
