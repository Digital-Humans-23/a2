#pragma once

#include "loco/robot/GeneralizedCoordinatesRobotRepresentation.h"
#include "loco/robot/Robot.h"
#include "loco/robot/RobotState.h"

namespace crl::loco {

/**
 * This class represents a generic limb (i.e. leg or arm). Each limb has an
 * origin RB, a bunch of links connected by joints, and an end effector RB.
 */
struct RobotLimb {
    // index of limb
    int limbIndex = -1;
    // this is the name of the limb
    std::string name;
    // and all limbs have an end effector
    std::shared_ptr<RB> eeRB = nullptr;
    // and this is a list of all the limb's joints - for easy access...
    std::vector<std::shared_ptr<RBJoint>> jointList;

    // this is the vector from the CoM of the trunk to the end effector; it
    // corresponds to a default step offset. Expressed in the coordinate frame
    // of the trunk...
    V3D defaultEEOffset;
    // this is ptr to ee we use for locomotion related tasks
    RBEndEffector *ee = nullptr;

    /**
     * constructor: looking for EE
     */
    RobotLimb(const std::string &name, const std::shared_ptr<RB> &eeRB, const std::shared_ptr<RB> &limbRoot) {
        this->name = name;
        this->eeRB = eeRB;
        this->ee = &eeRB->rbProps.endEffectorPoints[0];

        std::shared_ptr<RB> tmpRB = eeRB;
        while (tmpRB != limbRoot) {
            jointList.insert(jointList.begin(), tmpRB->pJoint);
            tmpRB = tmpRB->pJoint->parent;
        }

        P3D eePos = ee->endEffectorOffset;
        defaultEEOffset = limbRoot->getLocalCoordinates(V3D(limbRoot->getWorldCoordinates(P3D()), eeRB->getWorldCoordinates(eePos)));
    }

    /**
     * this corresponds to the hip or shoulder joint...
     */
    std::shared_ptr<RBJoint> getJointToTrunk() {
        return jointList[0];
    }

    P3D getEEWorldPos() const {
        return eeRB->getWorldCoordinates(this->ee->endEffectorOffset);
    }

    V3D getEEWorldVel() const {
        return eeRB->getVelocityForPoint_local(this->ee->endEffectorOffset);
    }

    bool isContact() const {
        return this->ee->inContact;
    }
};

/**
 * Each legged robot has a set of limbs and a trunk.
 */
class LeggedRobot : public Robot {
private:
    // we will assume that the trunk is just one rigid body for now, and that it
    // is the root of the robot...
    std::shared_ptr<RB> trunk = nullptr;
    std::vector<std::shared_ptr<RobotLimb>> limbs;

    // it's useful to store standing pose as a nominal state
    // if it is not specified then just set initial state of robot (but it might
    // be dangerous...)
    RobotState standingState;

public:
    explicit LeggedRobot(const char *filePath, const char *statePath = nullptr, bool loadVisuals = true);

    ~LeggedRobot() override = default;

    std::shared_ptr<RB> getTrunk();

    void addLimb(const std::string &name, const std::shared_ptr<RB> &eeRB);

    void addLimb(const std::string &name, const std::string &eeRBName);

    int getLimbCount() const;

    std::shared_ptr<RobotLimb> getLimb(uint i) const;

    /**
     * Search the limb corresponding to the queried name
     */
    std::shared_ptr<RobotLimb> getLimbByName(const std::string &name) const;
};

}  // namespace crl::loco
