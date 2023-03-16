#pragma once

#include <crl-basic/utils/mathUtils.h>

namespace crl::loco {

/**
 * This class acts as a container for the state information (position,
 * orientation, velocity and angular velocity - all of them stored in world
 * coordinates) of a rigid body.
 */

struct RBState {
    // the position of the center of mass of the rigid body, in world coords. In
    // local coordinates this corresponds to the point (0,0,0) aka origin.
    P3D pos = P3D(0, 0, 0);
    // its orientation - rotates from local coordinate frame to world coordinate
    // frame
    Quaternion orientation = Quaternion::Identity();
    // the velocity of the center of mass, in world coords
    V3D velocity = V3D(0, 0, 0);
    // and finally, the angular velocity in world coords
    V3D angularVelocity = V3D(0, 0, 0);

    /**
     * Default constructor - populate the data members using safe values..
     */
    RBState() = default;

    /**
     * A copy constructor.
     */
    RBState(const RBState &other);

    /**
     * A assign operator
     */
    RBState &operator=(const RBState &other);

    bool operator==(const RBState &other) const;

    /**
     * Default destructor.
     */
    ~RBState() = default;
};

}  // namespace crl::loco
