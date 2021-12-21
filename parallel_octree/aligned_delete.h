#pragma once

#include <new>

template <size_t Alignment>
struct aligned_delete final
{
	void operator()(void* ptr) const noexcept
	{
		operator delete (ptr, std::align_val_t(Alignment));
	}
};
