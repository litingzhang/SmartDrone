#pragma once

#include <cstdint>
#include <mutex>

struct MoveGoal {
    uint8_t frame{0};
    float x{0};
    float y{0};
    float z{1.2f};
    float yaw{0};
    float maxV{0.6f};
    uint32_t seq{0};
};

class GoalCache {
public:
    void Set(const MoveGoal& goal)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_goal = goal;
        m_hasGoal = true;
    }

    bool Get(MoveGoal* outGoal) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_hasGoal || outGoal == nullptr) {
            return false;
        }
        *outGoal = m_goal;
        return true;
    }

private:
    mutable std::mutex m_mutex;
    bool m_hasGoal{false};
    MoveGoal m_goal{};
};
