#include "loco/robot/RobotState.h"

#include "loco/robot/Robot.h"

namespace crl::loco {

RobotState::RobotState(const RobotState &other) {
    rootQ = other.rootQ;
    rootPos = other.rootPos;
    rootVel = other.rootVel;
    rootAngVel = other.rootAngVel;
    headingAxis = other.headingAxis;
    joints = other.joints;
}

RobotState::RobotState(int jCount, int aJCount) {
    joints.resize(jCount);
}

RobotState::RobotState(const Robot &robot, bool useDefaultAngles) {
    robot.populateState(*this, useDefaultAngles);
}

bool RobotState::operator==(const RobotState &other) const {
    if (getJointCount() != other.getJointCount()) {
        //			Logger::consolePrint("jCount: %d vs %d\n",
        // getJointCount(), other.getJointCount());
        return false;
    }

    if (V3D(getPosition(), other.getPosition()).norm() > 1e-10) {
        //			Logger::consolePrint("pos: %lf %lf %lf vs %lf
        //%lf %lf\n", 				getPosition().x(),
        // getPosition().y(), getPosition().z(),
        // other.getPosition().x(), other.getPosition().y(),
        // other.getPosition().z());
        return false;
    }

    Quaternion q1 = getOrientation();
    Quaternion q2 = other.getOrientation();

    if (!sameRotation(q1, q2)) {
        //	Logger::consolePrint("orientation: %lf %lf %lf %lf vs %lf %lf
        //%lf %lf\n", q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(),
        // q2.v.y(), q2.v.z());
        return false;
    }

    if ((getVelocity() - other.getVelocity()).norm() > 1e-10) {
        //			Logger::consolePrint("vel: %lf %lf %lf vs %lf
        //%lf %lf\n", 				getVelocity().x(),
        // getVelocity().y(), getVelocity().z(),
        // other.getVelocity().x(), other.getVelocity().y(),
        // other.getVelocity().z());
        return false;
    }

    if ((getAngularVelocity() - other.getAngularVelocity()).norm() > 1e-10) {
        //			Logger::consolePrint("ang vel: %lf %lf %lf vs
        //%lf %lf %lf\n", getAngularVelocity().x(),
        // getAngularVelocity().y(), getAngularVelocity().z(),
        // other.getAngularVelocity().x(), other.getAngularVelocity().y(),
        // other.getAngularVelocity().z());
        return false;
    }

    for (int i = 0; i < getJointCount(); i++) {
        if ((getJointRelativeAngVelocity(i) - other.getJointRelativeAngVelocity(i)).norm() > 1e-10) {
            //				Logger::consolePrint("joint %d
            // ang vel: %lf %lf %lf vs %lf %lf %lf\n", i,
            //					getJointRelativeAngVelocity(i).x(),
            // getJointRelativeAngVelocity(i).y(),
            // getJointRelativeAngVelocity(i).z(),
            //					other.getJointRelativeAngVelocity(i).x(),
            // other.getJointRelativeAngVelocity(i).y(),
            // other.getJointRelativeAngVelocity(i).z());
            return false;
        }

        Quaternion q1 = getJointRelativeOrientation(i);
        Quaternion q2 = other.getJointRelativeOrientation(i);

        if (!sameRotation(q1, q2)) {
            //				Logger::consolePrint("joint %d
            // orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", i, q1.s,
            // q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(),
            // q2.v.z());
            return false;
        }
    }

    return true;
}

void RobotState::writeToFile(const char *fName) {
    if (fName == nullptr)
        throwError("cannot write to a file whose name is nullptr!");

    FILE *fp = fopen(fName, "w");

    if (fp == nullptr)
        throwError("cannot open the file \'%s\' for reading...", fName);

    V3D velocity = getVelocity();
    Quaternion orientation = getOrientation();
    V3D angVelocity = getAngularVelocity();
    P3D position = getPosition();

    // double heading = getHeading();
    // setHeading(0);

    fprintf(fp,
            "# order is:\n# Heading Axis\n# Heading\n# Position\n# "
            "Orientation\n# Velocity\n# AngularVelocity\n\n# Relative "
            "Orientation\n# Relative Angular Velocity\n#----------------\n\n# "
            "Heading Axis\n %lf %lf %lf\n# Heading\n%lf\n\n",
            headingAxis[0], headingAxis[1], headingAxis[2], getHeading());

    fprintf(fp, "%lf %lf %lf\n", position.x, position.y, position.z);
    fprintf(fp, "%lf %lf %lf %lf\n", orientation.w(), orientation.x(), orientation.y(), orientation.z());
    fprintf(fp, "%lf %lf %lf\n", velocity[0], velocity[1], velocity[2]);
    fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);

    fprintf(fp, "# number of joints\n%d\n\n", getJointCount());

    for (int i = 0; i < getJointCount(); i++) {
        orientation = getJointRelativeOrientation(i);
        angVelocity = getJointRelativeAngVelocity(i);
        fprintf(fp, "%lf %lf %lf %lf\n", orientation.w(), orientation.x(), orientation.y(), orientation.z());
        fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);
    }

    fclose(fp);
    // now restore the state of this reduced state...
    // setHeading(heading);
}

void RobotState::readFromFile(const char *fName) {
    if (fName == nullptr)
        throwError("cannot read a file whose name is nullptr!");

    FILE *fp = fopen(fName, "r");
    if (fp == nullptr)
        throwError("cannot open the file \'%s\' for reading...", fName);

    double temp1, temp2, temp3, temp4;

    char line[100];

    // read the heading first...
    double heading;
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &headingAxis[0], &headingAxis[1], &headingAxis[2]);

    readValidLine(line, 100, fp);
    sscanf(line, "%lf", &heading);

    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    setPosition(P3D(temp1, temp2, temp3));
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
    setOrientation(Quaternion(temp1, temp2, temp3, temp4).normalized());
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    setVelocity(V3D(temp1, temp2, temp3));
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    setAngularVelocity(V3D(temp1, temp2, temp3));

    int jCount = 0;

    readValidLine(line, 100, fp);
    sscanf(line, "%d", &jCount);
    joints.resize(jCount);

    for (int i = 0; i < jCount; i++) {
        readValidLine(line, 100, fp);
        sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
        setJointRelativeOrientation(Quaternion(temp1, temp2, temp3, temp4).normalized(), i);
        readValidLine(line, 100, fp);
        sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
        setJointRelativeAngVelocity(V3D(temp1, temp2, temp3), i);
    }

    // now set the heading...
    setHeading(heading);

    fclose(fp);
}

// setting the heading...
void RobotState::setHeading(double heading) {
    // this means we must rotate the angular and linear velocities of the COM,
    // and augment the orientation
    Quaternion oldHeading = Quaternion::Identity(), newHeading = Quaternion::Identity(), qRoot = Quaternion::Identity();
    // get the current root orientation, that contains information regarding the
    // current heading
    qRoot = getOrientation();
    // get the twist about the vertical axis...
    oldHeading = computeHeading(qRoot, headingAxis);
    // now we cancel the initial twist and add a new one of our own choosing
    newHeading = getRotationQuaternion(heading, headingAxis) * oldHeading.inverse();
    // add this component to the root.
    setOrientation(newHeading * qRoot);
    // and also update the root velocity and angular velocity
    setVelocity(newHeading * getVelocity());
    setAngularVelocity(newHeading * getAngularVelocity());
}

}  // namespace crl::loco