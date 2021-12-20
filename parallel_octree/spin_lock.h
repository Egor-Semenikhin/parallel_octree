#pragma once

#include <cstdint>
#include <atomic>
#include <mutex>
#include <cassert>

class spin_lock final
{
private:
	std::atomic<uint32_t> _flag = 0;

public:
	void lock() noexcept
	{
		while (!try_lock());
	}

	void unlock() noexcept
	{
		assert(_flag == 1);
		_flag.store(0);
	}

	bool try_lock() noexcept
	{
		uint32_t expected = 0;
		return _flag.compare_exchange_strong(expected, 1u);
	}
};
