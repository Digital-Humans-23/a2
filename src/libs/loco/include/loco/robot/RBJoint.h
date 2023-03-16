#pragma once

#include <crl-basic/utils/mathUtils.h>
#include <crl-basic/utils/utils.h>

#include <memory>
#include <string>

#include "loco/robot/RB.h"

namespace crl::loco {

enum class RBJointControlMode {
    PASSIVE_MODE = 0,
    POSITION_MODE,
    VELOCITY_MODE,
    FORCE_MODE,
};

enum class RBJointType {
    REVOLUTE = 0,
    FIXED = 1,
};

/* forward declaration */
class RB;

/**
 * This class is used to implements hinge joints that allow relative rotation
 * between the parent and the child only about a given axis
 */
class RBJoint {
public:
    // the name of the joint
    std::string name;
    // unique index of the joint
    int jIndex = -1;
    // parent rigid body
    std::shared_ptr<RB> parent = nullptr;
    // this is the location of the joint on the parent - expressed in the
    // parent's local coordinates
    P3D pJPos = P3D(0, 0, 0);
    // this is the child link
    std::shared_ptr<RB> child = nullptr;
    // this is the location of the joint on the child - expressed in the child's
    // local coordinates
    P3D cJPos = P3D(0, 0, 0);
    // local coordinates of the rotation axis. Since the child and parent only
    // rotate relative to each other about this joint, the local coordinates for
    // the rotation axis are the same in parent and child frame
    V3D rotationAxis = V3D(0, 0, 1);

    // joint limits
    bool jointAngleLimitsActive = false;
    bool jointSpeedLimitActive = false;
    bool jointTorqueLimitActive = false;
    double minAngle = 0, maxAngle = 0;
    double maxSpeed = 40;    // rad/s
    double maxTorque = 100;  // N.m

    // keep a 'default' angle value... useful for applications that need a
    // regularizer...
    double defaultJointAngle = 0;

    // joint type
    RBJointType type = RBJointType::REVOLUTE;
    // joint control mode
    RBJointControlMode controlMode = RBJointControlMode::PASSIVE_MODE;

    // control signal
    double desiredControlTorque = 0;
    double desiredControlPosition = 0;
    double desiredControlSpeed = 0;

    // for position mode
    double motorKp = 12000.0;
    double motorKd = 0.1;
    double motorFeedback = 0;

    // selected in GUI
    bool selected = false;

public:
    /** Default constructor */
    RBJoint() = default;

    /** Default destructor */
    ~RBJoint() = default;

    /**
     * Returns the world position of the joint
     */
    P3D getWorldPosition() const;

    /**
     * computes the relative orientation between the parent and the child rigid
     * bodies
     */
    Quaternion computeRelativeOrientation() const;

    /**
     * This method is used to fix the errors in the joints (i.e. project state
     * of child such that joint configuration is consistent). The state of the
     * parent does not change.
     */
    void fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities);

    /**
     * this value ranges from -pi to pi
     */
    double getCurrentJointAngle() const;
};

}  // namespace crl::loco
