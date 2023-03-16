#pragma once

#include <crl-basic/utils/mathUtils.h>
#include <crl-basic/utils/utils.h>

#include "loco/robot/RB.h"
#include "loco/robot/RBJoint.h"
#include "loco/robot/RBLoader.h"
#include "loco/robot/Robot.h"

namespace crl::loco {

/**
 * This abstract class has a container for robots, rigid bodies and joints and
 * can be considered as a "world" where every rbs lives. For physics simulation,
 * derived class of RBEngine implements the physics world based on physics
 * engine. For real robots, derived class of RBEngine corresponds to a network
 * of robots.
 */

class RBEngine {
public:
    // this is a list of all robots in the world. robot has a list of rigid body
    // that are also governed by physics.
    std::vector<std::shared_ptr<Robot>> robots;
    // this is a list of all rigid bodies that are not part of robot.
    std::vector<std::shared_ptr<RB>> rbs;
    // and we'll keep a list of all the joints in the world as well - they
    // impose constraints on the relative movement between the rigid bodies they
    // connect
    std::vector<std::shared_ptr<RBJoint>> joints;

public:
    // the constructor
    RBEngine() = default;

    // the destructor
    virtual ~RBEngine() = default;

    virtual void addRobotToEngine(const std::shared_ptr<Robot> &robot);

    virtual void addRigidBodyToEngine(const std::shared_ptr<RB> &rb);

    virtual void addJointToEngine(const std::shared_ptr<RBJoint> &j);

    /**
     * This function loads rigid bodies and joints from rbs (or URDF...) file.
     * However, we don't build robot class instance here, rather just populate
     * rbEngine's rbs and joints list. This might be useful for loading a rigid
     * body with specific rbProps or loading multiple rigid bodies at once.
     */
    virtual void loadRBsFromFile(const char *fName);

    /**
     * This method marks end effector contacted with ground, other object,
     * Override this function for your own implementation
     * e.g. contact sensor reading, contact force > 0, or collision detection
     * algorithm...
     */
    virtual void markEEContacts(double threshold = 0.01);

    /**
     * Advance one timestep. This is a pure virtual function that needs to be
     * implemented in derived class.
     */
    virtual void step(double dt) = 0;

    /**
     * Apply force f to rb's point p where p is a point expressed in rb's body
     * frame. This is a pure virtual function so derived class must override and
     * implement this function. If it is not applicable, override this function
     * and make it private.
     */
    virtual void applyForceTo(const std::shared_ptr<RB> &rb, const V3D &f, const P3D &p) = 0;
};

}  // namespace crl::loco
