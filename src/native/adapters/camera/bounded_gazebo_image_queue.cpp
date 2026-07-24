#include "adapters/camera/bounded_gazebo_image_queue.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

namespace SmartDrone::Adapters::Camera {

struct BoundedGazeboImageQueue::Impl {
    struct Cell {
        std::atomic<std::size_t> sequence{0};
        GazeboRawImage image;
    };

    explicit Impl(std::size_t requestedCapacity)
        : capacity(requestedCapacity > 1 ? requestedCapacity : 2),
          cells(std::make_unique<Cell[]>(capacity))
    {
        for (std::size_t index = 0; index < capacity; ++index) {
            cells[index].sequence.store(index, std::memory_order_relaxed);
        }
    }

    bool TryEnqueue(const GazeboRawImage &image);
    bool TryDequeue(GazeboRawImage &image);

    const std::size_t capacity;
    std::unique_ptr<Cell[]> cells;
    alignas(64) std::atomic<std::size_t> enqueuePosition{0};
    alignas(64) std::atomic<std::size_t> dequeuePosition{0};
};

bool BoundedGazeboImageQueue::Impl::TryEnqueue(
    const GazeboRawImage &image)
{
    std::size_t position = enqueuePosition.load(std::memory_order_relaxed);
    Cell *cell = nullptr;
    while (true) {
        cell = &cells[position % capacity];
        const std::size_t sequence =
            cell->sequence.load(std::memory_order_acquire);
        const std::intptr_t difference =
            static_cast<std::intptr_t>(sequence) -
            static_cast<std::intptr_t>(position);
        if (difference == 0 && enqueuePosition.compare_exchange_weak(
                                   position, position + 1,
                                   std::memory_order_relaxed)) {
            break;
        }
        if (difference < 0) {
            return false;
        }
        position = enqueuePosition.load(std::memory_order_relaxed);
    }
    cell->image = image;
    cell->sequence.store(position + 1, std::memory_order_release);
    return true;
}

bool BoundedGazeboImageQueue::Impl::TryDequeue(GazeboRawImage &image)
{
    std::size_t position = dequeuePosition.load(std::memory_order_relaxed);
    Cell *cell = nullptr;
    while (true) {
        cell = &cells[position % capacity];
        const std::size_t sequence =
            cell->sequence.load(std::memory_order_acquire);
        const std::intptr_t difference =
            static_cast<std::intptr_t>(sequence) -
            static_cast<std::intptr_t>(position + 1);
        if (difference == 0 && dequeuePosition.compare_exchange_weak(
                                   position, position + 1,
                                   std::memory_order_relaxed)) {
            break;
        }
        if (difference < 0) {
            return false;
        }
        position = dequeuePosition.load(std::memory_order_relaxed);
    }
    image = std::move(cell->image);
    cell->sequence.store(position + capacity, std::memory_order_release);
    return true;
}

BoundedGazeboImageQueue::BoundedGazeboImageQueue(std::size_t capacity)
    : m_impl(std::make_unique<Impl>(capacity))
{
}

BoundedGazeboImageQueue::~BoundedGazeboImageQueue() = default;

bool BoundedGazeboImageQueue::TryEnqueue(const GazeboRawImage &image)
{
    return m_impl->TryEnqueue(image);
}

bool BoundedGazeboImageQueue::TryDequeue(GazeboRawImage &image)
{
    return m_impl->TryDequeue(image);
}

std::size_t BoundedGazeboImageQueue::Size() const
{
    const std::size_t enqueued =
        m_impl->enqueuePosition.load(std::memory_order_acquire);
    const std::size_t dequeued =
        m_impl->dequeuePosition.load(std::memory_order_acquire);
    return enqueued >= dequeued ? enqueued - dequeued : 0;
}

std::size_t BoundedGazeboImageQueue::Capacity() const
{
    return m_impl->capacity;
}

} // namespace SmartDrone::Adapters::Camera
