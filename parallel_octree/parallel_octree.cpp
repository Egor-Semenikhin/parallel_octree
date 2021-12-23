#include "parallel_octree.h"

#include "relative_ptr.h"

#include <span>
#include <memory_resource>

#define NOINLINE __declspec(noinline)
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
static constexpr uint32_t GC_HINT_FLAG = 0x80000000u;

struct parallel_octree::node
{
};

struct parallel_octree::tree final : public node
{
	relative_ptr<node> Children[8];
	uint32_t GCHint = 0;
};

struct parallel_octree::leaf final : public node
{
	uint32_t Count = 0;
	uint32_t GCHint = 0;
	uint32_t Indices[13] = {};
	relative_ptr<leaf_extension> Next;
};

struct parallel_octree::leaf_extension final
{
	uint32_t Indices[15] = {};
	relative_ptr<leaf_extension> Next;
};

template <bool Synchronized>
class parallel_octree::traverser_common
{
private:
	octree_allocator<>& _allocator;
	octree_allocator<>::local_part& _allocatorLocalPart;

protected:
	traverser_common(parallel_octree& owner, uint32_t workerIndex)
		: _allocator (owner._allocator)
		, _allocatorLocalPart (owner._allocator.get_local_part(workerIndex))
	{
	}

	template <typename TNode>
	TNode* allocate_node()
	{
		return _allocator.allocate<TNode, Synchronized>(_allocatorLocalPart);
	}

	template <typename TNode>
	void deallocate_node(TNode& node)
	{
		_allocator.deallocate(_allocatorLocalPart, node);
	}

	node* allocate_node(bool isTree)
	{
		return isTree ? static_cast<node*>(allocate_node<tree>()) : static_cast<node*>(allocate_node<leaf>());
	}

	void add_item(leaf& currentLeaf, uint32_t index)
	{
		uint32_t offset;

		if constexpr (Synchronized)
		{
			offset = reinterpret_cast<std::atomic<uint32_t>&>(currentLeaf.Count)++;
		}
		else
		{
			offset = currentLeaf.Count++;
		}

		if (offset < uint32_t(std::size(currentLeaf.Indices)))
			[[likely]]
		{
			currentLeaf.Indices[offset] = index;
			return;
		}

		offset -= uint32_t(std::size(currentLeaf.Indices));

		relative_ptr<leaf_extension>* prevPtr = &currentLeaf.Next;

		while (true)
		{
			leaf_extension* extension = prevPtr->get();

			if (!extension)
			{
				extension = allocate_node<leaf_extension>();

				if constexpr (Synchronized)
				{
					leaf_extension* expected = nullptr;
					if (!prevPtr->compare_exchange(expected, extension))
						[[unlikely]]
					{
						assert(expected);
						deallocate_node(*extension);
						extension = expected;
					}
				}
				else
				{
					*prevPtr = extension;
				}
			}

			if (offset < uint32_t(std::size(extension->Indices)))
				[[likely]]
			{
				extension->Indices[offset] = index;
				break;
			}

			offset -= uint32_t(std::size(extension->Indices));
			prevPtr = &extension->Next;
		}
	}

	static void set_gc_hint(uint32_t& value, uint32_t depth)
	{
		const uint32_t gcHint = GC_HINT_FLAG + depth;
		if constexpr (Synchronized)
		{
			reinterpret_cast<std::atomic<uint32_t>&>(value).store(gcHint);
		}
		else
		{
			value = gcHint;
		}
	}

	void remove_item(leaf& currentLeaf, uint32_t index, uint32_t depth)
	{
		uint32_t count;

		if constexpr (Synchronized)
		{
			count = reinterpret_cast<std::atomic<uint32_t>&>(currentLeaf.Count).load();
		}
		else
		{
			count = currentLeaf.Count;
		}

		set_gc_hint(currentLeaf.GCHint, depth);

		for (uint32_t i = 0, max = std::min(uint32_t(std::size(currentLeaf.Indices)), count); i < max; ++i)
		{
			if (currentLeaf.Indices[i] == index)
			{
				currentLeaf.Indices[i] = InvalidIndex;
				return;
			}
		}

		assert(count > uint32_t(std::size(currentLeaf.Indices)));
		count -= uint32_t(std::size(currentLeaf.Indices));

		leaf_extension* extension = currentLeaf.Next.get();
		assert(extension);

		while (true)
		{
			for (uint32_t i = 0, max = std::min(uint32_t(std::size(extension->Indices)), count); i < max; ++i)
			{
				if (extension->Indices[i] == index)
				{
					extension->Indices[i] = InvalidIndex;
					return;
				}
			}

			assert(count > uint32_t(std::size(extension->Indices)));
			count -= uint32_t(std::size(extension->Indices));

			extension = extension->Next.get();
			assert(extension);
		}
	}

	node* add_octant(uint32_t sizeLog, uint32_t depth, tree& currentTree, uint32_t octantIndex)
	{
		relative_ptr<node>& child = currentTree.Children[octantIndex];
		node* currentNode = child.get();

		if (currentNode)
			[[likely]]
		{
			return currentNode;
		}

		return allocate_octant(child, depth != sizeLog);
	}

private:
	NOINLINE node* allocate_octant(relative_ptr<node>& child, bool isTree)
	{
		node* currentNode = allocate_node(isTree);

		if constexpr (Synchronized)
		{
			node* expected = nullptr;
			if (!child.compare_exchange(expected, currentNode))
				[[unlikely]]
			{
				assert(expected != nullptr);
				if (isTree)
				{
					deallocate_node(static_cast<tree&>(*currentNode));
				}
				else
				{
					deallocate_node(static_cast<leaf&>(*currentNode));
				}
				currentNode = expected;
			}
		}
		else
		{
			child = currentNode;
		}

		return currentNode;
	}
};

template <bool Synchronized>
class parallel_octree::traverser_add final : private traverser_common<Synchronized>
{
private:
	shape_data _shapeData;
	uint32_t _sizeLog;

public:
	traverser_add(parallel_octree& owner, uint32_t workerIndex, const shape_data& shapeData)
		: traverser_common<Synchronized> (owner, workerIndex)
		, _shapeData (shapeData)
		, _sizeLog (owner._sizeLog)
	{
	}

	void traverse(const aabb& aabbNode, uint32_t depth, node& currentNode)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			traverser_common<Synchronized>::add_item(static_cast<leaf&>(currentNode), _shapeData.Index);
			return;
		}

		tree& currentTree = static_cast<tree&>(currentNode);
		const point centre = calculate_centre(aabbNode);

		depth += 1;

		traverse(aabb_0(aabbNode, centre), depth, currentTree, 0);
		traverse(aabb_1(aabbNode, centre), depth, currentTree, 1);
		traverse(aabb_2(aabbNode, centre), depth, currentTree, 2);
		traverse(aabb_3(aabbNode, centre), depth, currentTree, 3);
		traverse(aabb_4(aabbNode, centre), depth, currentTree, 4);
		traverse(aabb_5(aabbNode, centre), depth, currentTree, 5);
		traverse(aabb_6(aabbNode, centre), depth, currentTree, 6);
		traverse(aabb_7(aabbNode, centre), depth, currentTree, 7);
	}

private:
	void traverse(const aabb& aabbNode, uint32_t depth, tree& currentTree, uint32_t octantIndex)
	{
		if (are_intersected(_shapeData, aabbNode))
			[[unlikely]]
		{
			traverse(aabbNode, depth, *traverser_common<Synchronized>::add_octant(_sizeLog, depth, currentTree, octantIndex));
		}
	}
};

template <bool Synchronized>
class parallel_octree::traverser_remove final : private traverser_common<Synchronized>
{
private:
	shape_data _shapeData;
	uint32_t _sizeLog;

public:
	traverser_remove(parallel_octree& owner, uint32_t workerIndex, const shape_data& shapeData)
		: traverser_common<Synchronized> (owner, workerIndex)
		, _shapeData (shapeData)
		, _sizeLog (owner._sizeLog)
	{
	}

	bool traverse(const aabb& aabbNode, uint32_t depth, node& currentNode)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			traverser_common<Synchronized>::remove_item(static_cast<leaf&>(currentNode), _shapeData.Index, depth);
			return true;
		}

		tree& currentTree = static_cast<tree&>(currentNode);
		const point centre = calculate_centre(aabbNode);

		depth += 1;

		bool markForGC = false;

		markForGC |= traverse(aabb_0(aabbNode, centre), depth, currentTree, 0);
		markForGC |= traverse(aabb_1(aabbNode, centre), depth, currentTree, 1);
		markForGC |= traverse(aabb_2(aabbNode, centre), depth, currentTree, 2);
		markForGC |= traverse(aabb_3(aabbNode, centre), depth, currentTree, 3);
		markForGC |= traverse(aabb_4(aabbNode, centre), depth, currentTree, 4);
		markForGC |= traverse(aabb_5(aabbNode, centre), depth, currentTree, 5);
		markForGC |= traverse(aabb_6(aabbNode, centre), depth, currentTree, 6);
		markForGC |= traverse(aabb_7(aabbNode, centre), depth, currentTree, 7);

		if (markForGC)
		{
			traverser_common<Synchronized>::set_gc_hint(currentTree.GCHint, depth - 1);
		}

		return markForGC || currentTree.GCHint != 0;
	}

private:
	bool traverse(const aabb& aabbNode, uint32_t depth, tree& currentTree, uint32_t octantIndex)
	{
		if (are_intersected(_shapeData, aabbNode))
			[[unlikely]]
		{
			node* currentNode = currentTree.Children[octantIndex].get();
			assert(currentNode);

			return traverse(aabbNode, depth, *currentNode);
		}
		return false;
	}
};

template <bool Synchronized>
class parallel_octree::traverser_move final : private traverser_common<Synchronized>
{
private:
	shape_move _shapeMove;
	uint32_t _sizeLog;

public:
	traverser_move(parallel_octree& owner, uint32_t workerIndex, const shape_move& shapeMove)
		: traverser_common<Synchronized> (owner, workerIndex)
		, _shapeMove (shapeMove)
		, _sizeLog (owner._sizeLog)
	{
	}

	bool traverse(const aabb& aabbNode, uint32_t depth, node& currentNode, bool intersectsOld, bool intersectsNew)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			if (intersectsOld && !intersectsNew)
			{
				traverser_common<Synchronized>::remove_item(static_cast<leaf&>(currentNode), _shapeMove.Index, depth);
				return true;
			}
			else if (intersectsNew && !intersectsOld)
			{
				traverser_common<Synchronized>::add_item(static_cast<leaf&>(currentNode), _shapeMove.Index);
				return false;
			}
		}

		tree& currentTree = static_cast<tree&>(currentNode);
		const point centre = calculate_centre(aabbNode);

		depth += 1;

		bool markForGC = false;

		markForGC |= traverse(aabb_0(aabbNode, centre), depth, currentTree, 0);
		markForGC |= traverse(aabb_1(aabbNode, centre), depth, currentTree, 1);
		markForGC |= traverse(aabb_2(aabbNode, centre), depth, currentTree, 2);
		markForGC |= traverse(aabb_3(aabbNode, centre), depth, currentTree, 3);
		markForGC |= traverse(aabb_4(aabbNode, centre), depth, currentTree, 4);
		markForGC |= traverse(aabb_5(aabbNode, centre), depth, currentTree, 5);
		markForGC |= traverse(aabb_6(aabbNode, centre), depth, currentTree, 6);
		markForGC |= traverse(aabb_7(aabbNode, centre), depth, currentTree, 7);

		if (markForGC)
		{
			traverser_common<Synchronized>::set_gc_hint(currentTree.GCHint, depth - 1);
		}

		return markForGC;
	}

private:
	bool traverse(const aabb& aabbNode, uint32_t depth, tree& currentTree, uint32_t octantIndex)
	{
		const bool intersectsOld = are_intersected(_shapeMove.aabbOld, aabbNode);
		const bool intersectsNew = are_intersected(_shapeMove.aabbNew, aabbNode);

		if (intersectsOld || intersectsNew)
			[[unlikely]]
		{
			return traverse(
				aabbNode, depth,
				*traverser_common<Synchronized>::add_octant(_sizeLog, depth, currentTree, octantIndex),
				intersectsOld, intersectsNew
				);
		}
		return false;
	}
};

class parallel_octree::traverser_gc_roots final
{
private:
	uint32_t _depth;
	std::pmr::vector<gc_root>& _roots;

public:
	traverser_gc_roots(uint32_t depth, std::pmr::vector<gc_root>& roots)
		: _depth(depth)
		, _roots(roots)
	{
	}

	void traverse(node& currentNode, uint32_t depth)
	{
		tree& currentTree = static_cast<tree&>(currentNode);

		if (currentTree.GCHint == 0)
			[[likely]]
		{
			return;
		}

		if (depth == _depth)
			[[unlikely]]
		{
			_roots.emplace_back(gc_root{ currentTree });
			return;
		}

		currentTree.GCHint = 0;
		depth += 1;

		for (size_t i = 0; i < std::size(currentTree.Children); ++i)
		{
			if (node* const child = currentTree.Children[i].get())
			{
				traverse(*child, depth);
			}
		}
	}
};

class parallel_octree::traverser_gc final
{
private:
	std::pmr::vector<chunk_pool<false>>& _pools;
	chunk_pool<false> _pool;
	uint32_t _sizeLog;
	uint32_t _count;

public:
	traverser_gc(parallel_octree& owner, std::pmr::vector<chunk_pool<false>>& pools)
		: _pools (pools)
		, _sizeLog (owner._sizeLog)
		, _count (0)
	{
	}

	bool traverse(node& currentNode, uint32_t depth)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			leaf& currentLeaf = static_cast<leaf&>(currentNode);
			if (currentLeaf.GCHint == 0)
			{
				return false;
			}
			return process_leaf(currentLeaf);
		}

		tree& currentTree = static_cast<tree&>(currentNode);

		if (currentTree.GCHint == 0)
		{
			return false;
		}

		depth += 1;
		currentTree.GCHint = 0;
		bool needGC = true;

		for (size_t i = 0; i < std::size(currentTree.Children); ++i)
		{
			relative_ptr<node>& childPtr = currentTree.Children[i];

			if (node* const child = childPtr.get())
				[[unlikely]]
			{
				if (!traverse(*child, depth))
					[[likely]]
				{
					needGC = false;
					continue;
				}

				if (_count == octree_allocator<>::ARRAY_SIZE)
				{
					_pools.emplace_back(std::move(_pool));
					assert(_pool.is_empty());
					_count = 0;
				}

				++_count;
				childPtr = nullptr;

				if (depth == _sizeLog)
				{
					_pool.add<false>(*static_cast<leaf*>(child));
				}
				else
				{
					_pool.add<false>(*static_cast<tree*>(child));
				}
			}
		}

		return needGC;
	}

	void finalize(parallel_octree& owner)
	{
		if (_count > 0)
		{
			_pools.emplace_back(std::move(_pool));
			_count = 0;
		}
		if (_pools.size() > 0)
		{
			owner._allocator.add_pools(std::move(_pools));
			assert(_pools.size() == 0);
		}
	}

private:
	bool process_leaf(leaf& currentLeaf)
	{
		currentLeaf.GCHint = 0;

		relative_ptr<leaf_extension>* nextPtr = &currentLeaf.Next;
		std::span<uint32_t> span(currentLeaf.Indices);
		uint32_t offset = 0;

		uint32_t count = currentLeaf.Count;
		uint32_t newCount = 0;

		const auto processIndex = [&offset, &span, &nextPtr, &newCount](uint32_t currentIndex)
		{
			if (currentIndex == InvalidIndex)
				[[unlikely]]
			{
				return;
			}

			if (offset == span.size())
				[[unlikely]]
			{
				assert(*nextPtr);
				leaf_extension& extension = *nextPtr->get();
				nextPtr = &extension.Next;
				span = std::span<uint32_t>(extension.Indices);
			}

			span[offset++] = currentIndex;
			++newCount;
		};

		for (uint32_t i = 0, max = std::min(count, uint32_t(std::size(currentLeaf.Indices))); i < max; ++i)
		{
			processIndex(currentLeaf.Indices[i]);
		}

		if (count > uint32_t(std::size(currentLeaf.Indices)))
			[[unlikely]]
		{
			assert(currentLeaf.Next);
			count -= uint32_t(std::size(currentLeaf.Indices));

			leaf_extension* extension = currentLeaf.Next.get();

			while (true)
			{
				for (uint32_t i = 0, max = std::min(count, uint32_t(std::size(extension->Indices))); i < max; ++i)
				{
					processIndex(extension->Indices[i]);
				}

				if (count <= uint32_t(std::size(extension->Indices)))
					[[likely]]
				{
					break;
				}

				count -= uint32_t(std::size(extension->Indices));
				assert(extension->Next);
				extension = extension->Next.get();
			}
		}

		currentLeaf.Count = newCount;
		return newCount == 0;
	}
};

parallel_octree::parallel_octree(uint32_t sizeLog, uint32_t bufferSize, uint32_t workersCount)
	: _allocator (bufferSize, workersCount)
	, _root (sizeLog > 0 ? static_cast<node*>(_allocator.allocate<tree, false>()) : static_cast<node*>(_allocator.allocate<leaf, false>()))
	, _sizeLog (sizeLog)
{
	static_assert(sizeof(tree) <= CACHE_LINE_SIZE);
	static_assert(sizeof(leaf) == CACHE_LINE_SIZE);
	static_assert(sizeof(leaf_extension) == CACHE_LINE_SIZE);
}

parallel_octree::~parallel_octree()
{
}

void parallel_octree::add_synchronized(const shape_data& shapeData, uint32_t workerIndex)
{
	traverser_add<true>(*this, workerIndex, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::remove_synchronized(const shape_data& shapeData, uint32_t workerIndex)
{
	traverser_remove<true>(*this, workerIndex, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::move_synchronized(const shape_move& shapeMove, uint32_t workerIndex)
{
	const aabb aabbInitial = initial_aabb();
	traverser_move<true>(*this, workerIndex, shapeMove).traverse(
		aabbInitial, 0, *_root,
		are_intersected(shapeMove.aabbOld, aabbInitial),
		are_intersected(shapeMove.aabbNew, aabbInitial)
		);
}

void parallel_octree::add_exclusive(const shape_data& shapeData)
{
	traverser_add<false>(*this, 0, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::remove_exclusive(const shape_data& shapeData)
{
	traverser_remove<false>(*this, 0, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::move_exclusive(const shape_move& shapeMove)
{
	const aabb aabbInitial = initial_aabb();
	traverser_move<false>(*this, 0, shapeMove).traverse(
		aabbInitial, 0, *_root,
		are_intersected(shapeMove.aabbOld, aabbInitial),
		are_intersected(shapeMove.aabbNew, aabbInitial)
		);
}

void parallel_octree::prepare_garbage_collection(std::pmr::vector<gc_root>& roots, uint32_t depth)
{
	assert(depth < _sizeLog);
	_allocator.prepare_gc();
	roots.clear();
	traverser_gc_roots(depth, roots).traverse(*_root, 0);
}

void parallel_octree::collect_garbage(gc_root root)
{
	tree& currentTree = root.Tree;
	assert(currentTree.GCHint != 0);

	const uint32_t depth = currentTree.GCHint & ~GC_HINT_FLAG;
	assert(depth < _sizeLog);

	char gcBuffer[8 * 1024];
	std::pmr::monotonic_buffer_resource bufferResource(gcBuffer, sizeof(gcBuffer));
	std::pmr::vector<chunk_pool<false>> pools{ std::pmr::polymorphic_allocator<chunk_pool<false>>(&bufferResource) };

	traverser_gc traverser(*this, pools);
	traverser.traverse(currentTree, depth);
	traverser.finalize(*this);
}

float parallel_octree::field_size() const
{
	return float(1 << _sizeLog);
}

parallel_octree::aabb parallel_octree::initial_aabb() const
{
	const float size = field_size();
	return { { 0, 0, 0 }, { size, size, size } };
}

parallel_octree::aabb parallel_octree::aabb_0(const aabb& aabb, const point& centre)
{
	return { aabb.Min, centre };
}

parallel_octree::aabb parallel_octree::aabb_1(const aabb& aabb, const point& centre)
{
	return {
		{ aabb.Min.X,   centre.Y, aabb.Min.Z },
		{   centre.X, aabb.Max.Y,   centre.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_2(const aabb& aabb, const point& centre)
{
	return {
		{   centre.X, aabb.Min.Y, aabb.Min.Z },
		{ aabb.Max.X,   centre.Y,   centre.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_3(const aabb& aabb, const point& centre)
{
	return {
		{   centre.X,   centre.Y, aabb.Min.Z },
		{ aabb.Max.X, aabb.Max.Y,   centre.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_4(const aabb& aabb, const point& centre)
{
	return {
		{ aabb.Min.X, aabb.Min.Y,   centre.Z },
		{   centre.X,   centre.Y, aabb.Max.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_5(const aabb& aabb, const point& centre)
{
	return {
		{ aabb.Min.X,   centre.Y,   centre.Z },
		{   centre.X, aabb.Max.Y, aabb.Max.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_6(const aabb& aabb, const point& centre)
{
	return {
		{   centre.X,   centre.Y,    centre.Z },
		{ aabb.Max.X, aabb.Max.Y,   centre.Z }
	};
}

parallel_octree::aabb parallel_octree::aabb_7(const aabb& aabb, const point& centre)
{
	return { centre, aabb.Max };
}

bool parallel_octree::are_intersected(const aabb& left, const aabb& right)
{
	const auto intersects = [](float min0, float max0, float min1, float max1)
	{
		if (max1 < min0)
		{
			return false;
		}
		if (max0 < min1)
		{
			return false;
		}
		return true;
	};

	return
		intersects(left.Min.X, left.Max.X, right.Min.X, right.Max.X) &&
		intersects(left.Min.Y, left.Max.Y, right.Min.Y, right.Max.Y) &&
		intersects(left.Min.Z, left.Max.Z, right.Min.Z, right.Max.Z);
}

bool parallel_octree::are_intersected(const shape_data& shape, const aabb& aabb)
{
	return are_intersected(shape.AABB, aabb);
}

parallel_octree::point parallel_octree::calculate_centre(const aabb& aabb)
{
	return { (aabb.Min.X + aabb.Max.X) * 0.5f, (aabb.Min.Y + aabb.Max.Y) * 0.5f, (aabb.Min.Z + aabb.Max.Z) * 0.5f };
}
