#include "common/tlv/goal_cache.h"

#include <atomic>
#include <memory>

struct GoalCache::Impl {
    std::shared_ptr<const MoveGoal> goal;
};

GoalCache::GoalCache()
    : m_impl(std::make_unique<Impl>())
{
}

GoalCache::~GoalCache() = default;

void GoalCache::Set(const MoveGoal &goal)
{
    std::shared_ptr<const MoveGoal> snapshot = std::make_shared<MoveGoal>(goal);
    std::atomic_store_explicit(&m_impl->goal, snapshot, std::memory_order_release);
}

bool GoalCache::Get(MoveGoal *outGoal) const
{
    if (outGoal == nullptr) {
        return false;
    }

    const auto snapshot =
        std::atomic_load_explicit(&m_impl->goal, std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }

    *outGoal = *snapshot;
    return true;
}
