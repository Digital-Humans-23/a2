#pragma once

#include <crl-basic/utils/mathUtils.h>
#include <crl-basic/utils/utils.h>

#include "loco/robot/RBUtils.h"

namespace crl::loco {

/* forward declaration */
class Robot;

class JointState {
public:
    Quaternion qRel = Quaternion::Identity();
    V3D angVelRel = V3D(0, 0, 0);
};

class RobotState {
private:
    Quaternion rootQ = Quaternion::Identity();
    P3D rootPos = P3D(0, 0, 0);
    V3D rootVel = V3D(0, 0, 0);
    V3D rootAngVel = V3D(0, 0, 0);

    // to compute headings, we need to know which axis defines it (the yaw)
    V3D headingAxis = RBGlobals::worldUp;

    std::vector<JointState> joints;

public:
    ~RobotState() {}

    RobotState(const RobotState &other);

    RobotState(int jCount = 0, int aJCount = 0);

    RobotState(const Robot &robot, bool useDefaultAngles = false);

    inline void setJointCount(int jCount) {
        joints.resize(jCount);
    }

    inline void setHeadingAxis(const V3D &v) {
        headingAxis = v;
    }

    inline V3D getHeadingAxis() {
        return headingAxis;
    }

    inline int getJointCount() const {
        return (int)joints.size();
    }

    inline P3D getPosition() const {
        return rootPos;
    }

    inline void setPosition(const P3D &p) {
        rootPos = p;
    }

    inline Quaternion getOrientation() const {
        return rootQ;
    }

    inline void setOrientation(const Quaternion &q) {
        rootQ = q;
    }

    inline V3D getVelocity() const {
        return rootVel;
    }

    inline void setVelocity(const V3D &v) {
        rootVel = v;
    }

    inline V3D getAngularVelocity() const {
        return rootAngVel;
    }

    inline void setAngularVelocity(const V3D &v) {
        rootAngVel = v;
    }

    inline Quaternion getJointRelativeOrientation(int jIndex) const {
        if ((uint)jIndex < joints.size())
            return joints[jIndex].qRel;
        //	exit(0);
        return Quaternion::Identity();
    }

    inline V3D getJointRelativeAngVelocity(int jIndex) const {
        if ((uint)jIndex < joints.size())
            return joints[jIndex].angVelRel;
        //	exit(0);
        return V3D(0, 0, 0);
    }

    inline void setJointRelativeOrientation(const Quaternion &q, int jIndex) {
        if ((uint)jIndex < joints.size())
            joints[jIndex].qRel = q;
        //	else
        //		exit(0);
    }

    inline void setJointRelativeAngVelocity(const V3D &w, int jIndex) {
        if ((uint)jIndex < joints.size())
            joints[jIndex].angVelRel = w;
        //	else
        //		exit(0);
    }

    inline double getHeading() {
        // first we need to get the current heading of the robot.
        return getRotationAngle(computeHeading(getOrientation(), headingAxis), headingAxis);
    }

    bool operator==(const RobotState &other) const;

    void writeToFile(const char *fName);

    void readFromFile(const char *fName);

    void setHeading(double heading);
};

}  // namespace crl::loco
