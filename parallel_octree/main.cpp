#include <iostream>
#include <random>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <memory_resource>

#include "parallel_octree.h"
#include "task_scheduler.h"

static float random_float(std::minstd_rand0& rand)
{
	return float(rand()) / rand.max();
}

static parallel_octree::aabb random_aabb(std::minstd_rand0& rand, float fieldSize, float size)
{
	const float sizeX = random_float(rand) * size;
	const float sizeY = random_float(rand) * size;
	const float sizeZ = random_float(rand) * size;

	const float x0 = random_float(rand) * (fieldSize - sizeX);
	const float y0 = random_float(rand) * (fieldSize - sizeY);
	const float z0 = random_float(rand) * (fieldSize - sizeZ);
	const float x1 = x0 + sizeX;
	const float y1 = y0 + sizeY;
	const float z1 = z0 + sizeZ;

	return {
		{ x0, y0, z0 },
		{ x1, y1, z1 }
	};
}

struct parallel_task final
{
	std::atomic<size_t> Count;
	std::mutex Mutex;
	std::condition_variable Conditional;
};

constexpr size_t count = 100000;

static void parallel_add()
{
	task_scheduler taskScheduler(std::thread::hardware_concurrency());

	parallel_octree octree(10, 256 * 1024 * 1024, taskScheduler.threads_count());
	std::minstd_rand0 rand;

	std::vector<parallel_octree::shape_data> shapes;
	shapes.reserve(count);

	for (uint32_t i = 0; i < count; ++i)
	{
		parallel_octree::shape_data shape;
		shape.AABB = random_aabb(rand, octree.field_size(), random_float(rand) + 0.1f);
		shape.Index = i;
		shapes.push_back(shape);
	}

	const auto time0 = std::chrono::high_resolution_clock::now();

	const size_t chinkSize = 80;
	const size_t taskCount = (count + chinkSize - 1) / chinkSize;

	{
		parallel_task task{ taskCount };

		for (size_t i = 0; i < taskCount; ++i)
		{
			taskScheduler.schedule_task(
				[&task, i, &octree, &shapes, chinkSize](uint32_t workerIndex)
				{
					try
					{
						for (size_t j = 0; j < chinkSize; ++j)
						{
							const size_t index = i * chinkSize + j;
							if (index < shapes.size())
							{
								octree.add_synchronized(shapes[index], workerIndex);
							}
						}
					}
					catch (const std::exception& excp)
					{
						std::cerr << "Exception: " << excp.what() << std::endl;
					}
					if (--task.Count == 0)
						[[unlikely]]
					{
						task.Conditional.notify_one();
					}
				}
			);
		}

		if (task.Count > 0)
		{
			std::unique_lock<std::mutex> lock(task.Mutex);
			task.Conditional.wait(lock, [&task]() { return task.Count == 0; });
		}
	}

	const auto time1 = std::chrono::high_resolution_clock::now();

	{
		parallel_task task{ taskCount };

		for (size_t i = 0; i < taskCount; ++i)
		{
			taskScheduler.schedule_task(
				[&task, i, &octree, &shapes, chinkSize](uint32_t workerIndex)
				{
					try
					{
						for (size_t j = 0; j < chinkSize; ++j)
						{
							const size_t index = i * chinkSize + j;
							if (index < shapes.size())
							{
								octree.remove_synchronized(shapes[index], workerIndex);
							}
						}
					}
					catch (const std::exception& excp)
					{
						std::cerr << "Exception: " << excp.what() << std::endl;
					}
					if (--task.Count == 0)
						[[unlikely]]
					{
						task.Conditional.notify_one();
					}
				}
			);
		}

		if (task.Count > 0)
		{
			std::unique_lock<std::mutex> lock(task.Mutex);
			task.Conditional.wait(lock, [&task]() { return task.Count == 0; });
		}
	}

	const auto time2 = std::chrono::high_resolution_clock::now();

	char gcBuffer[4 * 1024];
	std::pmr::monotonic_buffer_resource bufferResource(gcBuffer, sizeof(gcBuffer));
	std::pmr::vector<parallel_octree::gc_root> roots{ std::pmr::polymorphic_allocator<parallel_octree::gc_root>(&bufferResource) };

	octree.prepare_garbage_collection(roots);

	const auto time3 = std::chrono::high_resolution_clock::now();

	{
		parallel_task task{ roots.size() };

		for (size_t i = 0; i < roots.size(); ++i)
		{
			taskScheduler.schedule_task(
				[&task, i, &octree, &roots](uint32_t workerIndex)
				{
					try
					{
						octree.collect_garbage(roots[i]);
					}
					catch (const std::exception& excp)
					{
						std::cerr << "Exception: " << excp.what() << std::endl;
					}
					if (--task.Count == 0)
						[[unlikely]]
					{
						task.Conditional.notify_one();
					}
				}
			);
		}

		if (task.Count > 0)
		{
			std::unique_lock<std::mutex> lock(task.Mutex);
			task.Conditional.wait(lock, [&task]() { return task.Count == 0; });
		}
	}

	const auto time4 = std::chrono::high_resolution_clock::now();

	{
		parallel_task task{ taskCount };

		for (size_t i = 0; i < taskCount; ++i)
		{
			taskScheduler.schedule_task(
				[&task, i, &octree, &shapes, chinkSize](uint32_t workerIndex)
				{
					try
					{
						for (size_t j = 0; j < chinkSize; ++j)
						{
							const size_t index = i * chinkSize + j;
							if (index < shapes.size())
							{
								octree.add_synchronized(shapes[index], workerIndex);
							}
						}
					}
					catch (const std::exception& excp)
					{
						std::cerr << "Exception: " << excp.what() << std::endl;
					}
					if (--task.Count == 0)
						[[unlikely]]
					{
						task.Conditional.notify_one();
					}
				}
			);
		}

		if (task.Count > 0)
		{
			std::unique_lock<std::mutex> lock(task.Mutex);
			task.Conditional.wait(lock, [&task]() { return task.Count == 0; });
		}
	}

	const auto time5 = std::chrono::high_resolution_clock::now();

	const std::chrono::duration<double> timeSpanAdd = std::chrono::duration_cast<std::chrono::duration<double>>(time1 - time0);
	const std::chrono::duration<double> timeSpanRemove = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);
	const std::chrono::duration<double> timeSpanRoots = std::chrono::duration_cast<std::chrono::duration<double>>(time3 - time2);
	const std::chrono::duration<double> timeSpanGC = std::chrono::duration_cast<std::chrono::duration<double>>(time4 - time3);
	const std::chrono::duration<double> timeSpanAddPlus = std::chrono::duration_cast<std::chrono::duration<double>>(time5 - time4);

	std::cout << "Parallel  add    " << timeSpanAdd.count() * 1000 << " ms." << std::endl;
	std::cout << "Parallel  remove " << timeSpanRemove.count() * 1000 << " ms." << std::endl;
	std::cout << "Parallel  roots  " << timeSpanRoots.count() * 1000 << " ms." << std::endl;
	std::cout << "Parallel  gc     " << timeSpanGC.count() * 1000 << " ms." << std::endl;
	std::cout << "Parallel  add+   " << timeSpanAddPlus.count() * 1000 << " ms." << std::endl;
}

static void exclusive_add()
{
	parallel_octree octree(10, 256 * 1024 * 1024, std::thread::hardware_concurrency());
	std::minstd_rand0 rand;

	std::vector<parallel_octree::shape_data> shapes;
	shapes.reserve(count);

	for (uint32_t i = 0; i < count; ++i)
	{
		parallel_octree::shape_data shape;
		shape.AABB = random_aabb(rand, octree.field_size(), random_float(rand) + 0.1f);
		shape.Index = i;
		shapes.push_back(shape);
	}

	const auto time0 = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < count; ++i)
	{
		octree.add_exclusive(shapes[i]);
	}

	const auto time1 = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < count; ++i)
	{
		octree.remove_exclusive(shapes[i]);
	}

	const auto time2 = std::chrono::high_resolution_clock::now();

	char gcBuffer[4 * 1024];
	std::pmr::monotonic_buffer_resource bufferResource(gcBuffer, sizeof(gcBuffer));
	std::pmr::vector<parallel_octree::gc_root> roots{ std::pmr::polymorphic_allocator<parallel_octree::gc_root>(&bufferResource) };

	octree.prepare_garbage_collection(roots);

	const auto time3 = std::chrono::high_resolution_clock::now();

	for (const parallel_octree::gc_root& root : roots)
	{
		octree.collect_garbage(root);
	}

	const auto time4 = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < count; ++i)
	{
		octree.add_exclusive(shapes[i]);
	}

	const auto time5 = std::chrono::high_resolution_clock::now();

	const std::chrono::duration<double> timeSpanAdd = std::chrono::duration_cast<std::chrono::duration<double>>(time1 - time0);
	const std::chrono::duration<double> timeSpanRemove = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);
	const std::chrono::duration<double> timeSpanRoots = std::chrono::duration_cast<std::chrono::duration<double>>(time3 - time2);
	const std::chrono::duration<double> timeSpanGC = std::chrono::duration_cast<std::chrono::duration<double>>(time4 - time3);
	const std::chrono::duration<double> timeSpanAddPlus = std::chrono::duration_cast<std::chrono::duration<double>>(time5 - time4);

	std::cout << "Exclusive add    " << timeSpanAdd.count() * 1000 << " ms." << std::endl;
	std::cout << "Exclusive remove " << timeSpanRemove.count() * 1000 << " ms." << std::endl;
	std::cout << "Exclusive roots  " << timeSpanRoots.count() * 1000 << " ms." << std::endl;
	std::cout << "Exclusive gc     " << timeSpanGC.count() * 1000 << " ms." << std::endl;
	std::cout << "Exclusive add+   " << timeSpanAddPlus.count() * 1000 << " ms." << std::endl;
}

int main()
{
	exclusive_add();
	//parallel_add();
}
