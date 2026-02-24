#pragma once
#include "goal_cache.hpp"
#include <string>
#include <iostream>

struct VehicleGate
{
    bool vioOk{true};
    bool offboardReady{true}; // 已开始持续 setpoint 流，且 PX4允许切offboard
};

class MavlinkHooks
{
public:
    virtual ~MavlinkHooks() = default;

    // 状态门控（从 PX4心跳/状态估出来）
    virtual VehicleGate gate() const = 0;

    // 命令执行：返回 true=成功，否则写 err
    virtual bool arm(std::string* err) = 0;
    virtual bool disarm(std::string* err) = 0;
    virtual bool setOffboard(std::string* err) = 0;
    virtual bool hold(std::string* err) = 0;
    virtual bool land(std::string* err) = 0;

    // MOVE：通常只更新目标点缓存，Offboard 50Hz loop 去消费
    virtual bool setMoveGoal(const MoveGoal& g, std::string* err) = 0;
};

class DummyHooks : public MavlinkHooks
{
public:
    VehicleGate gate() const override { return m_gate; }

    bool arm(std::string* err) override { (void)err; std::cout << "[HOOK] arm\n"; return true; }
    bool disarm(std::string* err) override { (void)err; std::cout << "[HOOK] disarm\n"; return true; }
    bool setOffboard(std::string* err) override { (void)err; std::cout << "[HOOK] offboard\n"; return true; }
    bool hold(std::string* err) override { (void)err; std::cout << "[HOOK] hold\n"; return true; }
    bool land(std::string* err) override { (void)err; std::cout << "[HOOK] land\n"; return true; }

    bool setMoveGoal(const MoveGoal& g, std::string* err) override
    {
        (void)err;
        std::cout << "[HOOK] move seq=" << g.seq
                  << " frame=" << (int)g.frame
                  << " x=" << g.x << " y=" << g.y << " z=" << g.z
                  << " yaw=" << g.yaw << " maxV=" << g.maxV << "\n";
        m_cache.set(g);
        return true;
    }

    GoalCache& cache() { return m_cache; }

private:
    VehicleGate m_gate{true, true};
    GoalCache m_cache;
};