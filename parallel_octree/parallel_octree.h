#pragma once

#include <cstdint>
#include <memory>

#include "octree_allocator.h"

class parallel_octree final
{
private:
	struct node;
	struct tree;
	struct leaf;
	struct leaf_extension;

	template <bool Synchronized>
	class traverser_common;

	template <bool Synchronized>
	class traverser_add;

	template <bool Synchronized>
	class traverser_remove;

	template <bool Synchronized>
	class traverser_move;

	class traverser_gc_roots;
	class traverser_gc;

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

	struct gc_root final
	{
		tree& Tree;
	};

private:
	octree_allocator<> _allocator;

	node* _root;
	uint32_t _sizeLog;

public:
	explicit parallel_octree(uint32_t sizeLog, uint32_t bufferSize, uint32_t workersCount);
	~parallel_octree();

	parallel_octree(const parallel_octree&) = delete;
	const parallel_octree& operator = (const parallel_octree&) = delete;

	float field_size() const;

	void add_synchronized(const shape_data& shapeData, uint32_t workerIndex);
	void remove_synchronized(const shape_data& shapeData, uint32_t workerIndex);
	void move_synchronized(const shape_move& shapeMove, uint32_t workerIndex);

	void add_exclusive(const shape_data& shapeData);
	void remove_exclusive(const shape_data& shapeData);
	void move_exclusive(const shape_move& shapeMove);

	void prepare_garbage_collection(std::pmr::vector<gc_root>& roots, uint32_t depth = 2);
	void collect_garbage(gc_root root);

private:
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
