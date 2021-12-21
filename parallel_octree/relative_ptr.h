#pragma once

#include <cstdint>
#include <limits>
#include <cassert>
#include <atomic>
#include <concepts>

template <typename T, typename TOffset = int32_t>
	requires std::same_as<TOffset, int32_t> || std::same_as<TOffset, int16_t> || std::same_as<TOffset, int8_t>
class relative_ptr final
{
private:
	std::atomic<TOffset> _offset;

public:
	relative_ptr(T* ptr)
		: _offset (get_diff(ptr))
	{
	}

	relative_ptr()
		: relative_ptr (nullptr)
	{
	}

	template <typename TOtherOffset>
	relative_ptr(const relative_ptr<T, TOtherOffset>& rhs)
		: relative_ptr (rhs.get())
	{
	}

	template <typename TOtherOffset>
	relative_ptr<T, TOffset>& operator = (const relative_ptr<T, TOtherOffset>& rhs)
	{
		_offset = get_diff(rhs.get());
		return *this;
	}

	relative_ptr<T, TOffset>& operator = (T* ptr)
	{
		_offset = get_diff(ptr);
		return *this;
	}

	operator bool() const
	{
		return _offset != 0;
	}

	template <typename TOtherOffset>
	bool operator == (const relative_ptr<T, TOtherOffset>& rhs) const
	{
		return get() == rhs.get();
	}

	template <typename TOtherOffset>
	bool operator != (const relative_ptr<T, TOtherOffset>& rhs) const
	{
		return !operator==(rhs);
	}

	bool operator == (T* rhs) const
	{
		return get() == rhs;
	}

	bool operator != (T* rhs) const
	{
		return !operator==(rhs);
	}

	T* get()
	{
		return from_diff(_offset);
	}

	bool compare_exchange(T*& expected, T* desired)
	{
		TOffset expectedDiff = get_diff(expected);
		const bool result = _offset.compare_exchange_strong(expectedDiff, get_diff(desired));
		expected = from_diff(expectedDiff);
		return result;
	}

private:
	TOffset get_diff(T* ptr)
	{
		if (ptr == nullptr)
		{
			return 0;
		}

		void* const thisPtr = this;
		void* const valuePtr = ptr;
		ptrdiff_t diff = static_cast<uint8_t*>(valuePtr) - static_cast<uint8_t*>(thisPtr);

		assert(diff != 0);
		assert(diff >= std::numeric_limits<TOffset>::min() && diff <= std::numeric_limits<TOffset>::max());

		return TOffset(diff);
	}

	T* from_diff(TOffset offset)
	{
		if (offset == 0)
		{
			return nullptr;
		}
		void* const thisPtr = this;
		void* const valuePtr = static_cast<uint8_t*>(thisPtr) + offset;
		return static_cast<T*>(valuePtr);
	}
};

template <typename T, typename TOffset>
bool operator == (T* lhs, const relative_ptr<T, TOffset>& rhs)
{
	return lhs == rhs.get();
}

template <typename T, typename TOffset>
bool operator != (T* lhs, const relative_ptr<T, TOffset>& rhs)
{
	return !operator==(lhs, rhs);
}
