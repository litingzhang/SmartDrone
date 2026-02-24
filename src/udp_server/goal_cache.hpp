#pragma once
#include <mutex>
#include <cstdint>

struct MoveGoal
{
    uint8_t frame{0};
    float x{0}, y{0}, z{1.2f};
    float yaw{0};
    float maxV{0.6f};
    uint32_t seq{0};
};

class GoalCache
{
public:
    void set(const MoveGoal& g)
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_goal = g;
        m_has = true;
    }

    bool get(MoveGoal* out) const
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_has) return false;
        *out = m_goal;
        return true;
    }

private:
    mutable std::mutex m_mtx;
    bool m_has{false};
    MoveGoal m_goal{};
};