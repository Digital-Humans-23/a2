#ifndef CRL_LOCO_LOCOMOTIONTRAJECTORYPLANNER_H
#define CRL_LOCO_LOCOMOTIONTRAJECTORYPLANNER_H

#include <crl-basic/gui/renderer.h>

#include "loco/LeggedLocomotionData.h"
#include "loco/LeggedRobot.h"
#include "loco/gait/ContactSchedule.h"

namespace crl::loco {

/**
 * Trajectory generator interface for locomotion tasks. We will assume a simple structure for the robot, namely a whole lotta legs connected to a trunk.
 * This generator can be queried for high level objectives, such as target position for the feet, position/orientation/velocity for the trunk, etc...
 */
class LocomotionTrajectoryPlanner {
public:
    // this is the moment in time when the current set of motions trajectories was generated...
    double planGenerationTime = 0;

    // this is the length (in seconds) of the planning horizon that we are considering here...
    double planningHorizon = 2.0;

    // for sampling purposes, this is the time step
    double sample_dt = 1 / 30.0;

    // debugging
    bool verbose = false;

protected:
    std::shared_ptr<LeggedRobot> robot = nullptr;
    std::shared_ptr<LeggedLocomotionData> data = nullptr;

    LeggedRobotCommand command;
    LeggedRobotGaitPlan gaitPlan;

public:
    /**
     * constructor
     */
    explicit LocomotionTrajectoryPlanner(const std::shared_ptr<LeggedRobot>& robot, const shared_ptr<LeggedLocomotionData>& data);

    virtual ~LocomotionTrajectoryPlanner() = default;

    virtual void plan();

    virtual void drawTrajectories(const gui::Shader& shader, double t) {}

    virtual void plotDebugInfo(double t) {}

protected:
    virtual void prepareForNewTimeStep();

    virtual void generateTrajectoriesFromCurrentState() = 0;

    virtual void populateData() = 0;
};

}  // namespace crl::loco

#endif  //CRL_LOCO_LOCOMOTIONTRAJECTORYPLANNER_H
