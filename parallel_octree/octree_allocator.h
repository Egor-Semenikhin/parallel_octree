#pragma once

#include <vector>
#include <memory>
#include <cassert>
#include <mutex>

#include "chunk_allocator.h"
#include "chunk_pool.h"
#include "aligned_delete.h"

template <size_t ChinkSize = CACHE_LINE_SIZE>
class alignas(CACHE_LINE_SIZE) octree_allocator final
{
public:
	struct local_part
	{
	};

	static constexpr uint32_t ARRAY_SIZE = 64;

private:
	struct alignas(CACHE_LINE_SIZE) local_part_impl final : local_part
	{
		chunk_pool<false, ChinkSize> Pool;
		bool PoolsNotEmpty = false;
	};

private:
	chunk_allocator<ChinkSize> _chunkAllocator;

	std::mutex _addPoolsMutex;
	std::vector<chunk_pool<false, ChinkSize>> _pools;
	std::unique_ptr<local_part_impl[]> _localParts;
	uint32_t _localPartsCount;

	alignas(CACHE_LINE_SIZE)
	size_t _poolOffset;

public:
	octree_allocator(uint32_t bufferSize, uint32_t localPartsCount)
		: _chunkAllocator (bufferSize)
		, _localParts (new local_part_impl[localPartsCount])
		, _localPartsCount (localPartsCount)
		, _poolOffset (0)
	{
	}

	local_part& get_local_part(uint32_t index) const
	{
		assert(index < _localPartsCount);
		return _localParts[index];
	}

	void prepare_gc()
	{
		if (!_poolOffset)
			[[unlikely]]
		{
			return;
		}

		const bool poolsNotEmpty = _pools.size() > _poolOffset;

		if (poolsNotEmpty)
			[[likely]]
		{
			std::vector<chunk_pool<false, ChinkSize>> newPools;
			newPools.reserve(_pools.size() - _poolOffset);
			for (auto iter = _pools.begin() + _poolOffset; iter != _pools.end(); ++iter)
			{
				newPools.emplace_back(std::move(*iter));
			}
			_pools = std::move(newPools);
			_poolOffset = 0;
		}
		else
		{
			_pools.clear();
		}

		for (uint32_t i = 0; i < _localPartsCount; ++i)
		{
			_localParts[i].PoolsNotEmpty = poolsNotEmpty;
		}
	}

	void add_pools(std::pmr::vector<chunk_pool<false, ChinkSize>>&& pools)
	{
		if (pools.size() == 0)
			[[unlikely]]
		{
			return;
		}

		const std::lock_guard<std::mutex> guard(_addPoolsMutex);

		assert(_poolOffset == 0);

		if (_pools.size() == 0)
			[[unlikely]]
		{
			for (uint32_t i = 0; i < _localPartsCount; ++i)
			{
				_localParts[i].PoolsNotEmpty = true;
			}
		}

		for (chunk_pool<false, ChinkSize>& pool : pools)
		{
			_pools.emplace_back(std::move(pool));
		}

		pools.clear();
	}

	template <typename T, bool Synchronized, typename ... TArgs>
	T* allocate(TArgs... args)
	{
		return _chunkAllocator.allocate<T, Synchronized, TArgs...>(std::forward<TArgs>(args)...);
	}

	template <typename T, bool Synchronized, typename ... TArgs>
	T* allocate(local_part& localPart, TArgs... args)
	{
		static_assert(sizeof(T) <= ChinkSize);
		static_assert(alignof(T) <= ChinkSize);
		return new (allocate_memory<Synchronized>(localPart)) T(std::forward<TArgs>(args)...);
	}

	template <typename T>
	void deallocate(local_part& localPart, T& obj)
	{
		local_part_impl& localPartImpl = static_cast<local_part_impl&>(localPart);
		localPartImpl.Pool.add<false>(obj);
	}

	template <bool Synchronized>
	void* allocate_memory()
	{
		return _chunkAllocator.allocate_memory<Synchronized>();
	}

	template <bool Synchronized>
	void* allocate_memory(local_part& localPart)
	{
		local_part_impl& localPartImpl = static_cast<local_part_impl&>(localPart);

		if (void* const memory = localPartImpl.Pool.try_allocate_memory<false>())
		{
			return memory;
		}

		if (localPartImpl.PoolsNotEmpty)
			[[likely]]
		{
			size_t poolOffset;
			if constexpr (Synchronized)
			{
				poolOffset = reinterpret_cast<std::atomic<size_t>&>(_poolOffset)++;
			}
			else
			{
				poolOffset = _poolOffset++;
			}

			if (poolOffset < _pools.size())
				[[likely]]
			{
				localPartImpl.Pool.merge<false>(_pools[poolOffset]);
				return localPartImpl.Pool.allocate_memory<false>();
			}
			else
			{
				localPartImpl.PoolsNotEmpty = false;
			}
		}

		char* const arrayMemory = static_cast<char*>(_chunkAllocator.allocate_memory<Synchronized>(ARRAY_SIZE));

		for (uint32_t i = 0; i < ARRAY_SIZE; ++i)
		{
			localPartImpl.Pool.add<false>(arrayMemory + i * ChinkSize);
		}

		return localPartImpl.Pool.allocate_memory<false>();
	}
};
