#pragma once

#include <crl-basic/utils/utils.h>

#include "loco/robot/RB.h"
#include "loco/robot/RBJoint.h"
#include "loco/robot/RBRenderer.h"
#include "loco/robot/RBUtils.h"
#include "loco/robot/RobotState.h"

namespace crl::loco {

/**
 * Robots are articulated figures (i.e. tree structures starting at a root)
 */
class Robot {
    friend class RobotState;
    friend class GeneralizedCoordinatesRobotRepresentation;
    friend class RBLoader;

public:
    // options
    bool showMeshes = true;
    bool showSkeleton = false;
    bool showJointAxes = false;
    bool showJointLimits = false;
    bool showJointAngles = false;
    bool showCollisionSpheres = false;
    bool showEndEffectors = false;
    bool showMOI = false;
    bool showCoordFrame = false;

protected:
    // root configuration
    std::shared_ptr<RB> root = nullptr;
    // keep lists of all the joints and all the RBs of the robot, for easy access
    std::vector<std::shared_ptr<RBJoint>> jointList;
    std::vector<std::shared_ptr<RB>> rbList;

    //useful to know which way is "forward" for this robot.
    V3D forward = V3D(0, 0, 1);

public:
    /** the constructor */
    Robot(const char *filePath, const char *statePath = nullptr, bool loadVisuals = true);

    /** the destructor */
    virtual ~Robot(void) = default;

    /**
     * This method computes the relative orientation of the parent and child
     * bodies of joint i.
     */
    static inline Quaternion getRelativeOrientationForJoint(const std::shared_ptr<const RBJoint> &joint) {
        return joint->computeRelativeOrientation();
    }

    /**
     * This method is used to get the relative angular velocities of the parent
     * and child bodies of joint i, expressed in parent's local coordinates.
     */
    static inline V3D getRelativeLocalCoordsAngularVelocityForJoint(const std::shared_ptr<const RBJoint> &joint) {
        // we will store wRel in the parent's coordinates, to get an orientation
        // invariant expression for it
        return joint->parent->getLocalCoordinates(V3D(joint->child->getAngularVelocity() - joint->parent->getAngularVelocity()));
    }

    static inline void setRelativeOrientationForJoint(const std::shared_ptr<RBJoint> &joint, const Quaternion &qRel) {
        joint->child->setOrientation(joint->parent->getOrientation() * qRel);
    }

    static inline void setRelativeLocalCoordsAngularVelocityForJoint(const std::shared_ptr<RBJoint> &joint, const V3D &relAngVel) {
        // assume relAngVel is stored in the parent's coordinate frame, to get
        // an orientation invariant expression for it
        joint->child->setAngularVelocity(joint->parent->getAngularVelocity() + joint->parent->getWorldCoordinates(relAngVel));
    }

    inline int getJointCount() {
        return (int)jointList.size();
    }

    inline std::shared_ptr<RBJoint> getJoint(int i) const {
        if (i < 0 || i > (int)jointList.size() - 1)
            return nullptr;
        return jointList[i];
    }

    inline int getRigidBodyCount() {
        return (int)jointList.size() + 1;
    }

    /**
     * returns a pointer to the ith rigid body of the virtual robot, where the
     * root is at 0, and the rest follow afterwards...
     */
    inline std::shared_ptr<RB> getRigidBody(int i) {
        if (i == 0)
            return root;
        if (i <= (int)jointList.size())
            return jointList[i - 1]->child;
        return nullptr;
    }

    /**
     * this method is used to return the current heading of the robot
     */
    inline Quaternion getHeading() {
        return computeHeading(root->getOrientation(), RBGlobals::worldUp);
    }

    /**
     * this method is used to return the current heading of the robot, specified
     * as an angle measured in radians
     */
    inline double getHeadingAngle() {
        return getRotationAngle(computeHeading(root->getOrientation(), RBGlobals::worldUp), RBGlobals::worldUp);
    }

    /**
     * this method is used to return a reference to the joint whose name is
     * passed as a parameter, or nullptr if it is not found.
     */
    inline std::shared_ptr<RBJoint> getJointByName(const char *jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0)
                return jointList[i];
        return nullptr;
    }

    /**
     * this method is used to return the index of the joint (whose name is
     * passed as a parameter) in the articulated figure hierarchy.
     */
    inline int getJointIndex(const char *jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0)
                return i;
        return -1;
    }

    /**
     * returns the root of the current articulated figure.
     */
    inline std::shared_ptr<RB> getRoot() {
        return root;
    }

    /**
     * This method is used to compute the center of mass of the robot.
     */
    P3D computeCOM() const;

    double getMass() const;

    /**
     * This method is used to compute the velocity of the center of mass of the articulated figure.
     */
    V3D computeCOMVelocity() const;

    /**
     * This method is used to compute moment of inertia tensor of articulated figure around its com.
     */
     Matrix3x3 computeWorldMOI() const;

    /**
     * this method is used to read the reduced state of the robot from the file
     */
    void loadReducedStateFromFile(const char *fName);

    /**
     * this method is used to write the reduced state of the robot to a file
     */
    void saveReducedStateToFile(const char *fName);

    /**
     * uses the state of the robot to populate the input
     */
    void populateState(RobotState &state, bool useDefaultAngles = false) const;

    /**
     * sets the state of the robot using the input
     */
    void setState(const RobotState &state);

    /**
     * sets the state of the robot using default joint configurations
     */
    void setDefaultState();

    /**
     * sets the state of the robot using all-zero joint configurations
     */
    void setZeroState();

    /**
     * makes sure the state of the robot is consistent with all the joint
     * types...
     */
    void fixJointConstraints();

    /**
     * this method updates the robot's root state and all connected rigid bodies
     */
    void setRootState(const P3D &position = P3D(0, 0, 0), const Quaternion &orientation = Quaternion::Identity(), const V3D &vel = V3D(0, 0, 0),
                      const V3D &angVel = V3D(0, 0, 0));

    /**
     * getter for current root state
     */
    void getRootState(P3D &position, Quaternion &orientation, V3D &vel, V3D &angVel);

    /**
     * this method is used to return a reference to the articulated figure's
     * rigid body whose name is passed in as a parameter, or nullptr if it is
     * not found.
     */
    std::shared_ptr<RB> getRBByName(const char *jName);

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    std::shared_ptr<RB> getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint, bool checkMeshes, bool checkSkeleton) {
        std::shared_ptr<RB> selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbList.size(); i++) {
            if (rbList[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint, checkMeshes, checkSkeleton)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbList[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }

    V3D getForward() const {
        return forward;
    }

    /**
     * draws the robot at its current state
     */
    void draw(const gui::Shader &rbShader, float alpha = 1.0);
};

}  // namespace crl::loco
