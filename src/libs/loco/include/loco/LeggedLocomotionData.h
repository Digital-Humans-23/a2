//
// Created by Dongho Kang on 20.01.22.
//

#ifndef CRL_LOCO_LEGGEDLOCOMOTIONDATA_H
#define CRL_LOCO_LEGGEDLOCOMOTIONDATA_H

#include "loco/gait/ContactSchedule.h"
#include "loco/planner/LimbMotionPlan.h"
#include "loco/robot/RobotState.h"
#include "loco/utils/MathUtils.h"

namespace crl::loco {

enum class QuadrupedalGaitType { STAND = 0, WALK, TROT, CRAWL, FLYING_TROT, PACE, PRONK, BOUND, HOP_TROT };

/**
 * user command
 */
struct LeggedRobotCommand {
    int gaitType = (int)QuadrupedalGaitType::STAND;
    double targetForwardSpeed = 0;
    double targetSidewaysSpeed = 0;
    double targetTurngingSpeed = 0;
    double targetBodyFrameHeight = 0.4;
    double trunkPitch = 0;
    double trunkRoll = 0;
    double stepWidthModifier = 0.7;
    double swingHeight = 0.1;
};

/**
 * sensor values
 */
struct LeggedRobotSensor {
    explicit LeggedRobotSensor(const std::shared_ptr<LeggedRobot>& robot) {
        GCRR gcrr(robot);
        dVector q, dq;
        gcrr.getQ(q);
        gcrr.getQDot(dq);
        jointPos = q.tail(robot->getJointCount()).eval();
        jointVel = dq.tail(robot->getJointCount()).eval();
        resize(jointTorque, robot->getJointCount());
        for (int i = 0; i < robot->getLimbCount(); i++) {
            contacts[robot->getLimb(i)->name] = robot->getLimb(i)->isContact();
            footSensors[robot->getLimb(i)->name] = 0;
        }
    }

    // accelerometer = Rbw * (a-g)
    V3D accelerometer = V3D(0, 0, 0);
    // gyroscope = Rbw * w
    V3D gyroscope = V3D(0, 0, 0);
    // imu often has its own mechanism to estimate orientation
    Quaternion imuOrientation = Quaternion::Identity();
    // joint position (rad)
    dVector jointPos;
    // joint velocity (rad/s)
    dVector jointVel;
    // joint torque (N.m)
    dVector jointTorque;
    // foot sensor (contact)
    std::map<std::string, bool> contacts;
    std::map<std::string, double> footSensors;  // can be used for monitoring purpose...
};

/**
 * contact state
 */
struct LeggedRobotContactState {
    explicit LeggedRobotContactState(const std::shared_ptr<LeggedRobot>& robot) {
        for (int i = 0; i < robot->getLimbCount(); i++) {
            isContact[robot->getLimb(i)->name] = robot->getLimb(i)->isContact();
            if (robot->getLimb(i)->isContact()) {
                contactProbability[robot->getLimb(i)->name] = 1.0;
                forceEstimation[robot->getLimb(i)->name] = robot->getLimb(i)->ee->contactForce;
            } else {
                contactProbability[robot->getLimb(i)->name] = 0.0;
                forceEstimation[robot->getLimb(i)->name] = robot->getLimb(i)->ee->contactForce;
            }
        }
    }

    std::map<std::string, bool> isContact;
    std::map<std::string, double> contactProbability;
    std::map<std::string, V3D> forceEstimation;
};

/**
 * full state and estimation info
 */
struct LeggedRobotState {
    explicit LeggedRobotState(const std::shared_ptr<LeggedRobot>& robot) {
        // init state
        basePosition = robot->getTrunk()->getWorldCoordinates(P3D());
        basePositionCov = V3D(0, 0, 0);
        baseOrientation = robot->getTrunk()->getOrientation();
        baseOrientationCov = V3D(0, 0, 0);
        baseVelocity = robot->getTrunk()->getVelocityForPoint_local(P3D());
        baseAngularVelocity = robot->getTrunk()->getAngularVelocity();
        GCRR gcrr(robot);
        dVector q, dq;
        gcrr.getQ(q);
        gcrr.getQDot(dq);
        jointPos = q.tail(robot->getJointCount()).eval();
        jointVel = dq.tail(robot->getJointCount()).eval();
        for (int i = 0; i < robot->getLimbCount(); i++) {
            feetPos[robot->getLimb(i)->name] = robot->getLimb(i)->getEEWorldPos();
            feetVel[robot->getLimb(i)->name] = robot->getLimb(i)->getEEWorldVel();
            contactPos[robot->getLimb(i)->name] = robot->getLimb(i)->getEEWorldPos();
            contactCov[robot->getLimb(i)->name] = V3D();
        }
    }

    P3D basePosition;
    V3D basePositionCov;
    V3D baseVelocity;
    V3D baseVelocityCov;
    Quaternion baseOrientation;
    V3D baseOrientationCov;
    V3D baseAngularVelocity;
    dVector jointPos;
    dVector jointVel;
    // useful for foot feedback
    std::map<std::string, P3D> feetPos;
    std::map<std::string, V3D> feetVel;

    // this is for debugging purpose
    std::map<std::string, P3D> contactPos;
    std::map<std::string, V3D> contactCov;
};

struct LeggedRobotGaitPlan {
    ContactSchedule cs;
    // body movement profiles
    // forward, sideways and turning
    Trajectory3D bodySpeedTrajectory;
    Trajectory1D bodyHeightTrajectory;
    Trajectory1D bodyPitchTrajectory;
};

struct LeggedRobotTrajectoryPlan {
    explicit LeggedRobotTrajectoryPlan(const std::shared_ptr<LeggedRobot>& robot) {
        robotForward = robot->getForward();
        trunkMotionTrajectory.addKnot(0, V3D(robot->getTrunk()->getWorldCoordinates(P3D())));
        trunkOrientationTrajectory.addKnot(0, V3D(0, 0, 0));
        for (int i = 0; i < robot->getLimbCount(); i++) {
            limbMotion.eePos[robot->getLimb(i)->name] = robot->getLimb(i)->getEEWorldPos();
        }
    }

    V3D robotForward = V3D(0, 0, 1);
    Trajectory3D trunkMotionTrajectory;
    Trajectory3D trunkOrientationTrajectory;  // rpy
    LimbMotionPlan limbMotion;
    // for some special cases where we want to track ground reaction force too.
    std::map<std::string, Trajectory3D> groundForce;

    // some interface functions
    P3D getTargetLimbEEPositionAtTime(const std::string& l, double t) {
        return limbMotion.getBezierEEPositionAtTime(l, t);
    }

    V3D getTargetLimbEEVelocityAtTime(const std::string& l, double t) {
        return limbMotion.getBezierEEVelocityAtTime(l, t);
    }

    V3D getTargetLimbEEAccelerationAtTime(const std::string& l, double t) {
        return limbMotion.getBezierEEAccelerationAtTime(l, t);
    }

    V3D getTargetLimbEEGroundForceAtTime(const std::string& l, double t) {
        if (groundForce.empty() || !groundForce[l].getKnotCount())
            return {0, 0, 0};
        return groundForce[l].evaluate_piecewise_constant(t, 1.0);
    }

    P3D getTargetTrunkPositionAtTime(double t) const {
        return P3D() + trunkMotionTrajectory.evaluate_linear(t);
    }

    V3D getTargetTrunkVelocityAtTime(double t, double dt = 1 / 30.0) const {
        P3D delta = getTargetTrunkPositionAtTime(t + dt) - getTargetTrunkPositionAtTime(t);
        return V3D(delta) / dt;
    }

    Quaternion getTargetTrunkOrientationAtTime(double t) const {
        return getOrientationFromRollPitchYawAngles(robotForward,
                                                    getTargetTrunkRollAtTime(t),   //
                                                    getTargetTrunkPitchAtTime(t),  //
                                                    getTargetTrunkHeadingAtTime(t));
    }

    V3D getTargetTrunkAngularVelocityAtTime(double t, double dt = 1 / 30.0) const {
        return estimateAngularVelocity(getTargetTrunkOrientationAtTime(t), getTargetTrunkOrientationAtTime(t + dt), dt);
    }

    double getTargetTrunkHeadingAtTime(double t) const {
        return trunkOrientationTrajectory.evaluate_linear(t)[2];
    }

    double getTargetTrunkPitchAtTime(double t) const {
        return trunkOrientationTrajectory.evaluate_linear(t)[1];
    }

    double getTargetTrunkRollAtTime(double t) const {
        return trunkOrientationTrajectory.evaluate_linear(t)[0];
    }

    ContactPhaseInfo getCPInformationFor(const std::string& l, double t) const {
        return limbMotion.cs.getContactPhaseInformation(l, t);
    }
};

struct LeggedRobotControlSignal {
    struct LeggedRobotJointControlSignal {
        RBJointControlMode mode = RBJointControlMode::PASSIVE_MODE;
        double desiredPos = 0;
        double desiredSpeed = 0;
        double desiredTorque = 0;
    };

    struct LeggedRobotFootControlSignal {
        P3D targetPos;
        V3D targetVel;
        V3D targetAcc;
        V3D targetGroundForce;

        // sometimes, it's useful to add some foot feedback...
        V3D footFeedbackKp;
        V3D footFeedbackKd;
    };

    explicit LeggedRobotControlSignal(const std::shared_ptr<LeggedRobot>& robot) {
        targetBasePos = robot->getTrunk()->getWorldCoordinates(P3D());
        targetBaseVel = robot->getTrunk()->getVelocityForPoint_local(P3D());
        targetBaseOrientation = robot->getTrunk()->getOrientation();
        targetBaseAngularVelocity = robot->getTrunk()->getAngularVelocity();
        targetBaseAcc = V3D();
        targetBaseAngularVelocity = V3D();
        jointControl.resize(robot->getJointCount());
        for (int i = 0; i < robot->getLimbCount(); i++) {
            footControl[robot->getLimb(i)->name].targetPos = robot->getLimb(i)->getEEWorldPos();
            footControl[robot->getLimb(i)->name].targetVel = robot->getLimb(i)->getEEWorldVel();
            footControl[robot->getLimb(i)->name].targetAcc = V3D();
            footControl[robot->getLimb(i)->name].targetGroundForce = V3D();
            footControl[robot->getLimb(i)->name].footFeedbackKp = V3D();
            footControl[robot->getLimb(i)->name].footFeedbackKd = V3D();
        }
    }

    P3D targetBasePos;
    V3D targetBaseVel;
    Quaternion targetBaseOrientation;
    V3D targetBaseAngularVelocity;
    V3D targetBaseAcc;
    V3D targetBaseAngularAcc;
    std::vector<LeggedRobotJointControlSignal> jointControl;
    std::map<std::string, LeggedRobotFootControlSignal> footControl;
};

/**
 * store every data required for locomotion task. this is abstract class.
 */
class LeggedLocomotionData {
protected:
    // command from user
    LeggedRobotCommand command;
    // sensor from robot
    LeggedRobotSensor sensor;
    // contact state
    LeggedRobotContactState contactState;
    // state estimation
    LeggedRobotState state;
    // gait plan
    LeggedRobotGaitPlan gaitPlan;
    // trajectory plan
    LeggedRobotTrajectoryPlan trajectoryPlan;
    // control signal
    LeggedRobotControlSignal control;

public:
    explicit LeggedLocomotionData(const std::shared_ptr<LeggedRobot>& robot)
        : sensor(robot), contactState(robot), state(robot), trajectoryPlan(robot), control(robot) {}

    virtual ~LeggedLocomotionData() = default;

    /* ---------------------------------------- Time Stamp ---------------------------------------- */

    virtual double getTimeStamp() const = 0;

    virtual void advanceInTime(double dt) = 0;

    /* ---------------------------------------- User Command ---------------------------------------- */

    virtual LeggedRobotCommand getCommand() {
        return command;
    }

    virtual void setCommand(const LeggedRobotCommand& command) {
        this->command = command;
    }

    /* ---------------------------------------- Sensor ---------------------------------------- */

    virtual LeggedRobotSensor getSensor() {
        return sensor;
    }

    virtual void setSensor(const LeggedRobotSensor& sensor) {
        this->sensor = sensor;
    }

    /* ---------------------------------------- Contact State -------------------------------- */

    virtual LeggedRobotContactState getContactState() {
        return contactState;
    }

    virtual void setContactState(const LeggedRobotContactState& contactState) {
        this->contactState = contactState;
    }

    /* ---------------------------------------- State ---------------------------------------- */

    virtual LeggedRobotState getLeggedRobotState() {
        return state;
    }

    /**
     * Legged robot state estimator should call this.
     */
    virtual void setLeggedRobotState(const LeggedRobotState& state) {
        this->state = state;
    }

    /* ---------------------------------------- Gait Plan ---------------------------------------- */

    virtual LeggedRobotGaitPlan getGaitPlan() {
        return gaitPlan;
    }

    /**
     *  Gait planner should call this.
     */
    virtual void setGaitPlan(const LeggedRobotGaitPlan& gaitPlan) {
        this->gaitPlan = gaitPlan;
    }

    /* ---------------------------------------- Trajectory Plan ---------------------------------------- */

    virtual LeggedRobotTrajectoryPlan getTrajectoryPlan() {
        return trajectoryPlan;
    }

    /**
     *  Trajectory planner should call this.
     */
    virtual void setTrajectoryPlan(const LeggedRobotTrajectoryPlan& trajectoryPlan) {
        this->trajectoryPlan = trajectoryPlan;
    }

    /* ---------------------------------------- Control ---------------------------------------- */

    virtual LeggedRobotControlSignal getControlSignal() {
        return control;
    }

    /**
     * Controller should call this.
     */
    virtual void setControlSignal(const LeggedRobotControlSignal& control) {
        this->control = control;
    }
};

/**
 * This is only for simulation. Some help function and members helpful for debugging.
 */
class LeggedLocomotionSimulationData : public LeggedLocomotionData {
private:
    // timestamp of this robot
    double timeStamp = 0;

    // for debugging
    ContactPlanManager cpm;

public:
    explicit LeggedLocomotionSimulationData(std::shared_ptr<LeggedRobot>& robot) : LeggedLocomotionData(robot) {}

    ~LeggedLocomotionSimulationData() override = default;

    double getTimeStamp() const override {
        return timeStamp;
    };

    void advanceInTime(double dt) override {
        timeStamp += dt;
    };

    /* ---------------------------------------- User Interface ---------------------------------------- */

    void visualizeContactSchedule(double time, double gridStartTime = -1, double dt_grid = 1 / 30.0, int nGridSteps = 30) {
        cpm.cs = gaitPlan.cs;
        cpm.visualizeContactSchedule(time, gridStartTime, dt_grid, nGridSteps);
    }

    void drawOptionMenu() {
        // gaits
        ImGui::RadioButton("Stand", &command.gaitType, int(QuadrupedalGaitType::STAND));
        ImGui::SameLine();
        ImGui::RadioButton("Crawl", &command.gaitType, int(QuadrupedalGaitType::CRAWL));
        ImGui::SameLine();
        ImGui::RadioButton("Trot", &command.gaitType, int(QuadrupedalGaitType::TROT));

        ImGui::RadioButton("Walk", &command.gaitType, int(QuadrupedalGaitType::WALK));
        ImGui::SameLine();
        ImGui::RadioButton("Pace", &command.gaitType, int(QuadrupedalGaitType::PACE));
        ImGui::SameLine();
        ImGui::RadioButton("Flying trot", &command.gaitType, int(QuadrupedalGaitType::FLYING_TROT));

        ImGui::RadioButton("Pronk", &command.gaitType, int(QuadrupedalGaitType::PRONK));
        ImGui::SameLine();
        ImGui::RadioButton("Bound", &command.gaitType, int(QuadrupedalGaitType::BOUND));
        ImGui::SameLine();
        ImGui::RadioButton("Hop trot", &command.gaitType, int(QuadrupedalGaitType::HOP_TROT));

        // body
        ImGui::InputDouble("Forward", &command.targetForwardSpeed, 0.1);
        ImGui::InputDouble("Sideways", &command.targetSidewaysSpeed, 0.1);
        ImGui::InputDouble("Turning", &command.targetTurngingSpeed, 0.1);
        ImGui::InputDouble("Height", &command.targetBodyFrameHeight, 0.01);

        ImGui::InputDouble("Body Pitch", &command.trunkPitch, 0.03);
        ImGui::InputDouble("Body Roll", &command.trunkRoll, 0.03);
        //        ImGui::InputDouble("Body Yaw", &trunkYaw, 0.03);

        ImGui::InputDouble("Step Width", &command.stepWidthModifier, 0.05);
        ImGui::InputDouble("Swing Height", &command.swingHeight, 0.01);
    }
};

}  // namespace crl::loco

#endif  //CRL_LOCO_LEGGEDLOCOMOTIONDATA_H
