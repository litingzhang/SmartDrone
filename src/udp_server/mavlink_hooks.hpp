#pragma once

#include "goal_cache.hpp"

#include <iostream>
#include <string>

struct VehicleGate {
    bool vioOk{true};
    bool offboardReady{true};
};

class MavlinkHooks {
public:
    virtual ~MavlinkHooks() = default;

    virtual VehicleGate GetGate() const = 0;
    virtual bool Arm(std::string* err) = 0;
    virtual bool Disarm(std::string* err) = 0;
    virtual bool SetOffboard(std::string* err) = 0;
    virtual bool Hold(std::string* err) = 0;
    virtual bool Land(std::string* err) = 0;
    virtual bool SetMoveGoal(const MoveGoal& goal, std::string* err) = 0;
};

class DummyHooks : public MavlinkHooks {
public:
    VehicleGate GetGate() const override { return m_gate; }

    bool Arm(std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] arm\n";
        return true;
    }

    bool Disarm(std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] disarm\n";
        return true;
    }

    bool SetOffboard(std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] offboard\n";
        return true;
    }

    bool Hold(std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] hold\n";
        return true;
    }

    bool Land(std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] land\n";
        return true;
    }

    bool SetMoveGoal(const MoveGoal& goal, std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] move seq=" << goal.seq << " frame=" << static_cast<int>(goal.frame)
                  << " x=" << goal.x << " y=" << goal.y << " z=" << goal.z
                  << " yaw=" << goal.yaw << " maxV=" << goal.maxV << "\n";
        m_cache.Set(goal);
        return true;
    }

    GoalCache& Cache() { return m_cache; }

private:
    VehicleGate m_gate{true, true};
    GoalCache m_cache;
};
