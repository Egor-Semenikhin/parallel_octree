#include <iostream>
#include <random>

#include "parallel_octree.h"

static float random_float(std::minstd_rand0& rand)
{
	return float(rand()) / rand.max();
}

static parallel_octree::aabb random_aabb(std::minstd_rand0& rand, float size)
{
	const float x0 = random_float(rand) * size;
	const float x1 = random_float(rand) * size;
	const float y0 = random_float(rand) * size;
	const float y1 = random_float(rand) * size;
	const float z0 = random_float(rand) * size;
	const float z1 = random_float(rand) * size;

	return {
		{ std::min(x0, x1), std::min(y0, y1), std::min(z0, z1) },
		{ std::max(x0, x1), std::max(y0, y1), std::max(z0, z1) }
	};
}

int main()
{
	parallel_octree octree(1, 1024);

	parallel_octree::shape_data shape;
	shape.AABB = { { 0.1f, 0.1f, 0.1f }, { 0.2f, 0.2f, 0.2f } };//random_aabb(rand, octree.field_size());
	shape.Index = 123;
	for (uint32_t i = 0; i < 15 + 15; ++i)
	{
		octree.add_synchronized(shape);
	}

	for (uint32_t i = 0; i < 15 + 15; ++i)
	{
		octree.remove_synchronized(shape);
	}
}
