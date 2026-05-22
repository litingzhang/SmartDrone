#pragma once

#include "runtime_command_hooks.h"

#include <string>

struct VehicleGate {
    bool vioOk{true};
    bool offboardReady{true};
};

class MavlinkHooks : public RuntimeCommandHook {
  public:
    ~MavlinkHooks() override = default;

    RuntimeCommandGate ReadCommandGate() const override
    {
        const VehicleGate gate = GetGate();
        RuntimeCommandGate runtimeGate{};
        runtimeGate.localizationReady = gate.vioOk;
        runtimeGate.guidedControlReady = gate.offboardReady;
        return runtimeGate;
    }

    bool ArmVehicle(std::string *errorMessage) override
    {
        return Arm(errorMessage);
    }
    bool DisarmVehicle(std::string *errorMessage) override
    {
        return Disarm(errorMessage);
    }
    bool StopVehicleImmediately(std::string *errorMessage) override
    {
        return EmergencyStop(errorMessage);
    }
    bool EnterGuidedControl(std::string *errorMessage) override
    {
        return SetOffboard(errorMessage);
    }
    bool HoldVehicle(std::string *errorMessage) override
    {
        return Hold(errorMessage);
    }
    bool EnterPositionControl(std::string *errorMessage) override
    {
        return Position(errorMessage);
    }
    bool LandVehicle(std::string *errorMessage) override
    {
        return Land(errorMessage);
    }
    bool ApplyMoveGoal(const MoveGoal &goal, std::string *errorMessage) override
    {
        return SetMoveGoal(goal, errorMessage);
    }

    virtual VehicleGate GetGate() const = 0;
    virtual bool Arm(std::string *errorMessage) = 0;
    virtual bool Disarm(std::string *errorMessage) = 0;
    virtual bool EmergencyStop(std::string *errorMessage) = 0;
    virtual bool SetOffboard(std::string *errorMessage) = 0;
    virtual bool Hold(std::string *errorMessage) = 0;
    virtual bool Position(std::string *errorMessage) = 0;
    virtual bool Land(std::string *errorMessage) = 0;
    virtual bool SetMoveGoal(const MoveGoal &goal, std::string *errorMessage) = 0;
};
