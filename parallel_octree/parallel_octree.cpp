#include "parallel_octree.h"

#include "relative_ptr.h"

#include <algorithm>

struct parallel_octree::node
{
};

struct parallel_octree::tree final : public node
{
	relative_ptr<node> Children[8];
};

struct parallel_octree::leaf final : public node
{
	uint32_t Count = 0;
	uint32_t Indices[14];
	relative_ptr<leaf_extension> Next;
};

struct parallel_octree::leaf_extension final
{
	uint32_t Indices[15];
	relative_ptr<leaf_extension> Next;
};

template <>
class parallel_octree::traverser<true>
{
protected:
	parallel_octree& _owner;

private:
	chunk_pool<false> _localPool;

protected:
	explicit traverser(parallel_octree& owner)
		: _owner (owner)
	{
	}

	~traverser()
	{
		_owner._chunkPool.merge<true>(_localPool);
	}

	template <typename TNode>
	TNode* allocate_node()
	{
		if (TNode* const node = _localPool.try_allocate<TNode, false>())
		{
			return node;
		}
		return _owner.allocate_node<TNode, true>();
	}

	template <typename TNode>
	void deallocate_node(TNode& node)
	{
		_localPool.add<false>(node);
	}
};

template <>
class parallel_octree::traverser<false>
{
protected:
	parallel_octree& _owner;

protected:
	explicit traverser(parallel_octree& owner)
		: _owner (owner)
	{
	}

	template <typename TNode>
	TNode* allocate_node()
	{
		return _owner.allocate_node<TNode, false>();
	}

	template <typename TNode>
	void deallocate_node(TNode& node)
	{
		_owner._chunkPool.add<false>(node);
	}
};

template <bool Synchronized>
class parallel_octree::traverser_common : protected traverser<Synchronized>
{
protected:
	explicit traverser_common(parallel_octree& owner)
		: traverser<Synchronized> (owner)
	{
	}

	node* allocate_node(bool isLeaf)
	{
		return isLeaf ?
			static_cast<node*>(traverser<Synchronized>::template allocate_node<leaf>()) :
			static_cast<node*>(traverser<Synchronized>::template allocate_node<tree>());
	}

	void add_item(leaf& currentLeaf, uint32_t index)
	{
		uint32_t offset;

		if constexpr (Synchronized)
		{
			static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
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
				extension = traverser<Synchronized>::template allocate_node<leaf_extension>();

				if constexpr (Synchronized)
				{
					leaf_extension* expected = nullptr;
					if (!prevPtr->compare_exchange(expected, extension))
						[[unlikely]]
					{
						assert(expected);
						traverser<Synchronized>::template deallocate_node(*extension);
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

	void remove_item(leaf& currentLeaf, uint32_t index)
	{
		uint32_t count;

		if constexpr (Synchronized)
		{
			static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
			count = reinterpret_cast<std::atomic<uint32_t>&>(currentLeaf.Count).load();
		}
		else
		{
			count = currentLeaf.Count;
		}

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

		const bool isTree = depth != sizeLog;
		currentNode = allocate_node(isTree);

		if constexpr (Synchronized)
		{
			node* expected = nullptr;
			if (!child.compare_exchange(expected, currentNode))
				[[unlikely]]
			{
				assert(expected != nullptr);
				if (isTree)
				{
					traverser<Synchronized>::deallocate_node(static_cast<tree&>(*currentNode));
				}
				else
				{
					traverser<Synchronized>::deallocate_node(static_cast<leaf&>(*currentNode));
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
	traverser_add(parallel_octree& owner, const shape_data& shapeData)
		: traverser_common<Synchronized> (owner)
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
	traverser_remove(parallel_octree& owner, const shape_data& shapeData)
		: traverser_common<Synchronized> (owner)
		, _shapeData (shapeData)
		, _sizeLog (owner._sizeLog)
	{
	}

	void traverse(const aabb& aabbNode, uint32_t depth, node& currentNode)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			traverser_common<Synchronized>::remove_item(static_cast<leaf&>(currentNode), _shapeData.Index);
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
		if (!are_intersected(_shapeData, aabbNode))
			[[likely]]
		{
			return;
		}

		node* currentNode = currentTree.Children[octantIndex].get();
		assert(currentNode);

		traverse(aabbNode, depth, *currentNode);
	}
};

template <bool Synchronized>
class parallel_octree::traverser_move final : private traverser_common<Synchronized>
{
private:
	shape_move _shapeMove;
	uint32_t _sizeLog;

public:
	traverser_move(parallel_octree& owner, const shape_move& shapeMove)
		: traverser_common<Synchronized> (owner)
		, _shapeMove (shapeMove)
		, _sizeLog (owner._sizeLog)
	{
	}

	void traverse(const aabb& aabbNode, uint32_t depth, node& currentNode, bool intersectsOld, bool intersectsNew)
	{
		if (depth == _sizeLog)
			[[unlikely]]
		{
			if (intersectsOld && !intersectsNew)
			{
				traverser_common<Synchronized>::remove_item(static_cast<leaf&>(currentNode), _shapeMove.Index);
			}
			else if (intersectsNew && !intersectsOld)
			{
				traverser_common<Synchronized>::add_item(static_cast<leaf&>(currentNode), _shapeMove.Index);
			}
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
		const bool intersectsOld = are_intersected(_shapeMove.aabbOld, aabbNode);
		const bool intersectsNew = are_intersected(_shapeMove.aabbNew, aabbNode);

		if (intersectsOld || intersectsNew)
			[[unlikely]]
		{
			traverse(
				aabbNode, depth,
				*traverser_common<Synchronized>::add_octant(_sizeLog, depth, currentTree, octantIndex),
				intersectsOld, intersectsNew
				);
		}
	}
};

parallel_octree::parallel_octree(uint32_t sizeLog, size_t bufferSize)
	: _chunkAllocator (bufferSize)
	, _root (allocate_node<false>(sizeLog > 0))
	, _sizeLog (sizeLog)
{
}

parallel_octree::~parallel_octree()
{
}

void parallel_octree::add_synchronized(const shape_data& shapeData)
{
	traverser_add<true>(*this, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::remove_synchronized(const shape_data& shapeData)
{
	traverser_remove<true>(*this, shapeData).traverse(initial_aabb(), 0, *_root);
}

void parallel_octree::move_synchronized(const shape_move& shapeMove)
{
	const aabb aabbInitial = initial_aabb();
	traverser_move<true>(*this, shapeMove).traverse(
		aabbInitial, 0, *_root,
		are_intersected(shapeMove.aabbOld, aabbInitial),
		are_intersected(shapeMove.aabbNew, aabbInitial)
		);
}

template <bool Synchronized>
parallel_octree::node* parallel_octree::allocate_node(bool isTree)
{
	return isTree ? static_cast<node*>(allocate_node<tree, Synchronized>()) : static_cast<node*>(allocate_node<leaf, Synchronized>());
}

template <typename TNode, bool Synchronized>
TNode* parallel_octree::allocate_node()
{
	if (TNode* const currentNode = _chunkPool.try_allocate<TNode, Synchronized>())
	{
		return currentNode;
	}
	return _chunkAllocator.allocate<TNode, Synchronized>();
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
