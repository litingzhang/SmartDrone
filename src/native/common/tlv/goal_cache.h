#pragma once

#include <cstdint>
#include <memory>

#include "tlv_protocol.h"

struct MoveGoal {
    uint8_t frame{0};
    bool isVelocity{false};
    bool isRcJoystick{false};
    float x{0};
    float y{0};
    float z{1.2f};
    float yaw{0};
    float vx{0};
    float vy{0};
    float vz{0};
    float yawRate{0};
    float throttleNorm{0};
    float yawNorm{0};
    float pitchNorm{0};
    float rollNorm{0};
    float maxV{0.6f};
    uint32_t seq{0};
};

class GoalCache {
  public:
    GoalCache();
    ~GoalCache();

    void Set(const MoveGoal &goal);
    bool Get(MoveGoal *outGoal) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
