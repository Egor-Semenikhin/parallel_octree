#pragma once

#include "cache_line.h"
#include "aligned_delete.h"

#include <cstdint>
#include <atomic>
#include <new>
#include <exception>
#include <memory>
#include <cassert>

template <size_t ChinkSize = CACHE_LINE_SIZE>
class alignas(CACHE_LINE_SIZE) chunk_allocator final
{
	static_assert(sizeof(size_t) == sizeof(std::atomic<size_t>));

private:
	const size_t _size;
	const std::unique_ptr<uint8_t[], aligned_delete<CACHE_LINE_SIZE>> _data;

	alignas(CACHE_LINE_SIZE) size_t _offset;

public:
	explicit chunk_allocator(size_t size)
		: _size((size + ChinkSize - 1) / ChinkSize * ChinkSize)
		, _data(static_cast<uint8_t*>(operator new (_size, std::align_val_t(CACHE_LINE_SIZE))))
		, _offset(0)
	{
		assert(std::atomic<size_t>().is_lock_free());
	}

	chunk_allocator(const chunk_allocator&) = delete;
	const chunk_allocator& operator = (const chunk_allocator&) = delete;

	template <typename T, bool Synchronized, typename ... TArgs>
	T* allocate(TArgs... args)
	{
		static_assert(sizeof(T) <= ChinkSize);
		static_assert(alignof(T) <= ChinkSize);
		return new (allocate_memory<Synchronized>()) T(std::forward<TArgs>(args)...);
	}

	template <bool Synchronized>
	void* allocate_memory(size_t count = 1)
	{
		size_t prevOffset;
		size_t currentOffset;

		const size_t size = ChinkSize * count;

		if constexpr (Synchronized)
		{
			prevOffset = reinterpret_cast<std::atomic<size_t>&>(_offset).fetch_add(size);
			currentOffset = prevOffset + size;
		}
		else
		{
			prevOffset = _offset;
			_offset = currentOffset = prevOffset + size;
		}

		if (currentOffset > _size)
			[[unlikely]]
		{
			throw std::bad_alloc();
		}

		return _data.get() + prevOffset;
	}
};
