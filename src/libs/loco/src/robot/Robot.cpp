#include "loco/robot/Robot.h"

#include "loco/robot/RBEngine.h"
#include "loco/robot/RBLoader.h"

namespace crl::loco {

Robot::Robot(const char *filePath, const char *statePath, bool loadVisuals) {
    // load robot from rbLoader
    RBLoader rbLoader(filePath, loadVisuals);
    rbLoader.populateRobot(*this);

    // set initial state
    // load robot state from rs file or
    // set default state based on joint default angle
    if (statePath && strlen(statePath) > 0) {
        loadReducedStateFromFile(statePath);
    } else {
        RobotState rs(*this, true);
        setState(rs);
    }
}

void Robot::populateState(RobotState &state, bool useDefaultAngles) const {
    // we'll push the root's state information - ugly code....
    state.setPosition(root->getWorldCoordinates(P3D()));
    state.setOrientation(root->getOrientation());
    state.setVelocity(root->getVelocityForPoint_local(P3D()));
    state.setAngularVelocity(root->getAngularVelocity());
    state.setHeadingAxis(RBGlobals::worldUp);

    state.setJointCount((int)jointList.size());

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!

    for (uint i = 0; i < jointList.size(); i++) {
        if (!useDefaultAngles) {
            state.setJointRelativeOrientation(getRelativeOrientationForJoint(jointList[i]), i);
            state.setJointRelativeAngVelocity(getRelativeLocalCoordsAngularVelocityForJoint(jointList[i]), i);
        } else {
            state.setJointRelativeAngVelocity(V3D(0, 0, 0), i);
            state.setJointRelativeOrientation(getRotationQuaternion(jointList[i]->defaultJointAngle, jointList[i]->rotationAxis), i);
        }
    }
}

void Robot::setDefaultState() {
    RobotState rs;
    populateState(rs, true);
    setState(rs);
}

void Robot::setZeroState() {
    RobotState rs;
    populateState(rs, true);
    for (uint i = 0; i < jointList.size(); i++) {
        rs.setJointRelativeAngVelocity(V3D(0, 0, 0), i);
        rs.setJointRelativeOrientation(getRotationQuaternion(0, jointList[i]->rotationAxis), i);
    }
    setState(rs);
}

void Robot::setState(const RobotState &state) {
    // kinda ugly code....
    root->setPosition(state.getPosition());
    root->setOrientation(state.getOrientation());
    root->setVelocity(state.getVelocity());
    root->setAngularVelocity(state.getAngularVelocity());

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!
    for (uint j = 0; j < jointList.size(); j++) {
        setRelativeOrientationForJoint(jointList[j], state.getJointRelativeOrientation((int)j).normalized());
        setRelativeLocalCoordsAngularVelocityForJoint(jointList[j], state.getJointRelativeAngVelocity((int)j));
        // and now set the linear position and velocity
        jointList[j]->fixJointConstraints(true, true, true, true);
    }
}

void Robot::fixJointConstraints() {
    for (size_t j = 0; j < jointList.size(); j++)
        jointList[j]->fixJointConstraints(true, true, true, true);
}

P3D Robot::computeCOM() const {
    P3D COM = root->getWorldCoordinates(P3D()) * root->rbProps.mass;
    double totalMass = root->rbProps.mass;

    for (uint i = 0; i < jointList.size(); i++) {
        totalMass += jointList[i]->child->rbProps.mass;
        COM += jointList[i]->child->getWorldCoordinates(P3D()) * jointList[i]->child->rbProps.mass;
    }

    return COM / totalMass;
}

double Robot::getMass() const {
    // compute the mass of the robot
    double mass = root->rbProps.mass;
    for (uint i = 0; i < jointList.size(); i++)
        mass += jointList[i]->child->rbProps.mass;
    return mass;
}

V3D Robot::computeCOMVelocity() const {
    V3D COMVel = root->getVelocityForPoint_local(P3D()) * root->rbProps.mass;
    double totalMass = root->rbProps.mass;

    for (uint i = 0; i < jointList.size(); i++) {
        totalMass += jointList[i]->child->rbProps.mass;
        COMVel += jointList[i]->child->getVelocityForPoint_local(P3D()) * jointList[i]->child->rbProps.mass;
    }

    return COMVel / totalMass;
}

Matrix3x3 Robot::computeWorldMOI() const {
    P3D com = computeCOM();
    Matrix3x3 MOIw = root->getWorldMOI(V3D(root->getWorldCoordinates(P3D()), com));
    for (const auto &j : jointList) {
        MOIw += j->child->getWorldMOI(V3D(j->child->getWorldCoordinates(P3D()), com));
    }
    return MOIw;
}

void Robot::setRootState(const P3D &position, const Quaternion &orientation, const V3D &vel, const V3D &angVel) {
    RobotState state(*this);
    populateState(state);
    state.setPosition(position);
    state.setOrientation(orientation);
    state.setVelocity(vel);
    state.setAngularVelocity(angVel);
    setState(state);
}

void Robot::getRootState(P3D &position, Quaternion &orientation, V3D &vel, V3D &angVel) {
    position = root->getWorldCoordinates(P3D());
    orientation = root->getOrientation().normalized();
    vel = root->getVelocityForPoint_local(P3D());
    angVel = root->getAngularVelocity();
}

void Robot::loadReducedStateFromFile(const char *fName) {
    RobotState state(*this);
    state.readFromFile(fName);
    setState(state);
}

void Robot::saveReducedStateToFile(const char *fName) {
    RobotState state(*this);
    state.writeToFile(fName);
}

std::shared_ptr<RB> Robot::getRBByName(const char *jName) {
    for (uint i = 0; i < jointList.size(); i++) {
        if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
            return jointList[i]->parent;
        if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
            return jointList[i]->child;
    }
    std::cout << "WARNING: Robot:getRBByName -> rigid body could not be found..." << std::endl;
    return nullptr;
}

void Robot::draw(const gui::Shader &rbShader, float alpha) {
    // Draw abstract view first
    if (showSkeleton)
        for (const auto &rb : rbList)
            RBRenderer::drawSkeletonView(rb, rbShader, showJointAxes, showJointLimits, showJointAngles, alpha);

    // Then draw meshes (because of blending)
    if (showMeshes)
        for (const auto &rb : rbList)
            RBRenderer::drawMeshes(rb, rbShader, alpha);

    // Then draw collsion spheres
    if (showCollisionSpheres)
        for (const auto &rb : rbList)
            RBRenderer::drawCollisionShapes(rb, rbShader);

    // Then draw end effectors
    if (showEndEffectors)
        for (const auto &rb : rbList)
            RBRenderer::drawEndEffectors(rb, rbShader);

    // and now MOIs
    if (showMOI)
        for (const auto &rb : rbList)
            RBRenderer::drawMOI(rb, rbShader);

    // and now coordinate frames
    if (showCoordFrame) {
        for (const auto &rb : rbList)
            RBRenderer::drawCoordFrame(rb, rbShader);
    }
}

}  // namespace crl::loco
