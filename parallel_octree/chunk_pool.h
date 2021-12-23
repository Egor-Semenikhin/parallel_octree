#pragma once

#include "cache_line.h"
#include "spin_lock.h"

namespace detail {
	template <bool Synchronized>
	class chunk_pool_base;

	template <>
	class chunk_pool_base<true>
	{
	private:
		spin_lock _spinLock;

	public:
		struct guard final : private std::lock_guard<spin_lock>
		{
			explicit guard(chunk_pool_base<true>& pool) noexcept
				: std::lock_guard<spin_lock>(pool._spinLock)
			{
			}
		};
	};

	template <>
	class chunk_pool_base<false>
	{
	public:
		struct guard final
		{
			explicit guard(chunk_pool_base<false>&) noexcept {}
		};
	};
}	// namespace detail

template <bool Synchronized, size_t ChinkSize = CACHE_LINE_SIZE>
class alignas(CACHE_LINE_SIZE) chunk_pool final : private detail::chunk_pool_base<Synchronized>
{
	static_assert(ChinkSize >= sizeof(void*));

private:
	struct header final
	{
		header* _next;
	};

private:
	header* _first = nullptr;

public:
	chunk_pool() noexcept {}

	template <bool PoolSynchronized>
	chunk_pool(chunk_pool<PoolSynchronized, ChinkSize>&& pool)
		: _first (pool.take<true>())
	{
	}

	chunk_pool(const chunk_pool<Synchronized, ChinkSize>&) = delete;
	const chunk_pool<Synchronized, ChinkSize>& operator = (const chunk_pool<Synchronized, ChinkSize>&) = delete;

	bool is_empty() const
	{
		return !_first;
	}

	template <typename T, bool DoSynchronize, typename ... TArgs>
	T* try_allocate(TArgs... args) noexcept
	{
		static_assert(sizeof(T) <= ChinkSize);
		if (void* const memory = try_allocate_memory())
			[[likely]]
		{
			return new (memory) T(std::forward<TArgs>(args)...);
		}
		return nullptr;
	}

	template <bool DoSynchronize>
	void* try_allocate_memory()
	{
		if constexpr (DoSynchronize)
		{
			const typename detail::chunk_pool_base<Synchronized>::guard guard(*this);
			return try_allocate();
		}
		else
		{
			return try_allocate();
		}
	}

	template <bool DoSynchronize>
	void* allocate_memory()
	{
		void* const memory = try_allocate_memory<DoSynchronize>();
		assert(memory);
		return memory;
	}

	template <bool DoSynchronize, typename T>
	void add(T& obj) noexcept
	{
		static_assert(sizeof(T) <= ChinkSize);
		obj.~T();
		add<DoSynchronize>(&obj);
	}

	template <bool DoSynchronize>
	void add(void* obj)
	{
		if constexpr (DoSynchronize)
		{
			const typename detail::chunk_pool_base<Synchronized>::guard guard(*this);
			do_add(obj);
		}
		else
		{
			do_add(obj);
		}
	}

	template <bool DoSynchronize, bool PoolSynchronized>
	void merge(chunk_pool<PoolSynchronized, ChinkSize>& pool)
	{
		header* poolHeader = reinterpret_cast<header*>(pool.take<DoSynchronize>());

		if (!poolHeader)
		{
			return;
		}

		header* last = poolHeader;

		while (last->_next)
		{
			last = last->_next;
		}

		if constexpr (DoSynchronize)
		{
			const typename detail::chunk_pool_base<Synchronized>::guard guard(*this);
			merge(poolHeader, last);
		}
		else
		{
			merge(poolHeader, last);
		}
	}

	template <bool DoSynchronize>
	header* take()
	{
		header* h;

		if constexpr (DoSynchronize)
		{
			const typename detail::chunk_pool_base<Synchronized>::guard guard(*this);
			h = _first;
			_first = nullptr;
		}
		else
		{
			h = _first;
			_first = nullptr;
		}

		return h;
	}

private:
	header* try_allocate()
	{
		if (!_first)
		{
			return nullptr;
		}

		header* const h = _first;
		_first = h->_next;

		return h;
	}

	void do_add(void* obj)
	{
		header* next = _first;
		_first = static_cast<header*>(obj);
		_first->_next = next;
	}

	void merge(header* first, header* last)
	{
		last->_next = _first;
		_first = first;
	}
};
