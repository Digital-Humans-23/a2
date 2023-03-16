#ifndef CRL_LOCO_LOCOMOTIONCONTROLLER_H
#define CRL_LOCO_LOCOMOTIONCONTROLLER_H

#include <imgui.h>

#include <memory>

#include "crl-basic/gui/shader.h"
#include "loco/LeggedLocomotionData.h"

namespace crl::loco {

/**
 * Abstract class for legged locomotion controllers.
 */
class LocomotionController {
public:
    // for debugging
    bool verbose = false;

protected:
    // Robot pointer
    std::shared_ptr<LeggedRobot> robot = nullptr;
    std::shared_ptr<LeggedLocomotionData> data = nullptr;

    // Plan
    double timer = 0;
    LeggedRobotTrajectoryPlan plan;

public:
    explicit LocomotionController(const std::shared_ptr<LeggedRobot>& robot, const std::shared_ptr<LeggedLocomotionData>& data)
        : robot(robot), data(data), plan(robot) {}

    virtual ~LocomotionController() = default;

    /**
     * Compute and apply control signal with timestep size dt.
     */
    virtual void computeAndApplyControlSignals(double dt) {
        prepareForControlStep(dt);
        computeControlSignals(dt);
        applyControlSignals(dt);
        populateData();
    }

    /**
     * Draw control options to ImGui.
     */
    virtual void drawOptionMenu() {}

    /**
     * Draw some useful information for debugging.
     * e.g. contact, velocity, acceleration etc.
     */
    virtual void drawDebugInfo(const gui::Shader& shader) = 0;

    /**
     * Plot some useful information for debugging.
     * e.g. joint torque etc.
     */
    virtual void plotDebugInfo() {}

protected:
    virtual void prepareForControlStep(double dt) {
        // update timer
        this->timer = data->getTimeStamp();
        // update robot state
        const LeggedRobotState& state = data->getLeggedRobotState();
        robot->setRootState(state.basePosition, state.baseOrientation, state.baseVelocity, state.baseAngularVelocity);
        GCRR gcrr(robot);
        dVector q, dq;
        gcrr.getQ(q);
        gcrr.getQDot(dq);
        for (int i = 0; i < robot->getJointCount(); i++) {
            q[i + 6] = state.jointPos[i];
            dq[i + 6] = state.jointVel[i];
        }
        gcrr.setQ(q);
        gcrr.setQDot(dq);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
        // update plan
        this->plan = data->getTrajectoryPlan();
        for (int i = 0; i < robot->getLimbCount(); i++)
            plan.limbMotion.eePos[robot->getLimb(i)->name] = robot->getLimb(i)->getEEWorldPos();
    }

    /*
     * control logic
     */
    virtual void computeControlSignals(double dt) = 0;

    /**
     * set desired joint targets to RBJoint.
     */
    virtual void applyControlSignals(double dt) = 0;

    virtual void populateData() = 0;
};

}  // namespace crl::loco

#endif  //CRL_LOCO_LOCOMOTIONCONTROLLER_H
