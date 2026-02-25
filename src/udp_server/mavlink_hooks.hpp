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
