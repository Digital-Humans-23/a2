#include "loco/planner/LocomotionTrajectoryPlanner.h"

#include "loco/BodyFrame.h"

namespace crl::loco {

LocomotionTrajectoryPlanner::LocomotionTrajectoryPlanner(const std::shared_ptr<LeggedRobot>& robot, const shared_ptr<LeggedLocomotionData>& data)
    : robot(robot), data(data) {}

void LocomotionTrajectoryPlanner::prepareForNewTimeStep() {
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
    planGenerationTime = data->getTimeStamp();

    // user command
    command = data->getCommand();
    gaitPlan = data->getGaitPlan();
}

void LocomotionTrajectoryPlanner::plan() {
    prepareForNewTimeStep();
    generateTrajectoriesFromCurrentState();
    populateData();
}

}  // namespace crl::loco
