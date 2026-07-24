#pragma once

#include "goal_cache.h"

#include <string>

struct RuntimeCommandGate {
    bool localizationReady{true};
    bool guidedControlReady{true};
};

class RuntimeCommandHook {
  public:
    virtual ~RuntimeCommandHook() = default;

    virtual RuntimeCommandGate ReadCommandGate() const = 0;
    virtual bool ArmVehicle(std::string *errorMessage) = 0;
    virtual bool DisarmVehicle(std::string *errorMessage) = 0;
    virtual bool StopVehicleImmediately(std::string *errorMessage) = 0;
    virtual bool EnterGuidedControl(std::string *errorMessage) = 0;
    virtual bool HoldVehicle(std::string *errorMessage) = 0;
    virtual bool EnterPositionControl(std::string *errorMessage) = 0;
    virtual bool LandVehicle(std::string *errorMessage) = 0;
    virtual bool IsLandingConfirmed() const = 0;
    virtual bool ApplyMoveGoal(const MoveGoal &goal, std::string *errorMessage) = 0;
};
