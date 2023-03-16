#pragma once

#include <crl-basic/utils/geoms.h>
#include <crl-basic/utils/mathUtils.h>

#include "loco/robot/RBJoint.h"
#include "loco/robot/RBProperties.h"
#include "loco/robot/RBState.h"
#include "loco/robot/RBUtils.h"

namespace crl::loco {

/* forward declaration */
class RBJoint;

/**
 * Rigidbody class.
 */
class RB {
public:
    // a list of properties for the rigid body
    RBProperties rbProps;
    // we will keep a list of the joints that have this rigid body as a parent
    // (child joints).
    std::vector<std::shared_ptr<RBJoint>> cJoints;
    // and the parent joint.
    std::shared_ptr<RBJoint> pJoint = nullptr;
    // name of the rigid body
    std::string name;

private:
    // state of the rigid body
    // this should be safe. always modify state with getter/setter.
    RBState state;

public:
    /**
     * Default constructor
     */
    RB() = default;

    /**
     * Default destructor
     */
    virtual ~RB() = default;

    /**
     *  returns the world coords moment of inertia of the rigid body.
     */
    Matrix3x3 getWorldMOI() const;

    /**
     * returns the world coords moment of inertia of the rigid body given offset vector. (parallel axis theorem)
     */
    Matrix3x3 getWorldMOI(const V3D &v) const;

    /**
     *  returns true if it is hit, false otherwise.
     */
    bool getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint, bool checkMeshes, bool checkSkeleton);

    /**
     * This method returns the coordinates of the point that is passed in as a
     * parameter(expressed in local coordinates), in world coordinates.
     */
    inline P3D getWorldCoordinates(const P3D &pLocal) const {
        // pWorld = pos + R * V3D(origin, pLocal)
        return state.pos + state.orientation * V3D(pLocal);
    }

    /**
     * This method returns the vector that is passed in as a parameter(expressed
     * in local coordinates), in world coordinates.
     */
    inline V3D getWorldCoordinates(const V3D &vLocal) const {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return state.orientation * vLocal;
    }

    /**
     * This method is used to return the local coordinates of the point that is
     * passed in as a parameter (expressed in global coordinates)
     */
    inline P3D getLocalCoordinates(const P3D &pWorld) {
        return P3D() + state.orientation.inverse() * (V3D(state.pos, pWorld));
    }

    /**
     * This method is used to return the local coordinates of the vector that is
     * passed in as a parameter (expressed in global coordinates)
     */
    inline V3D getLocalCoordinates(const V3D &vWorld) {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return state.orientation.inverse() * vWorld;
    }

    /**
     * This method returns the world coordinates velocity of a point that is
     * passed in as a parameter. The point is expressed in local coordinates,
     * and the resulting velocity will be expressed in world coordinates.
     */
    inline V3D getVelocityForPoint_local(const P3D &pLocal) const {
        // we need to compute the vector r, from the origin of the body to the
        // point of interest
        V3D r = V3D(pLocal);
        // the velocity is given by omega x r + v. omega and v are already
        // expressed in world coordinates, so we need to express r in world
        // coordinates first.
        return state.angularVelocity.cross(getWorldCoordinates(r)) + state.velocity;
    }

    /**
     * This method returns the world coordinates velocity of a point that is
     * passed in as a parameter. The point is expressed in world coordinates,
     * and the resulting velocity will be expressed in world coordinates.
     */
    inline V3D getVelocityForPoint_global(const P3D &pWorld) const {
        // we need to compute the vector r, from the origin of the body to the
        // point of interest
        V3D r(state.pos, pWorld);
        // the velocity is given by omega x r + v. omega and v are already
        // expressed in world coordinates, so we need to express r in world
        // coordinates first.
        return state.angularVelocity.cross(r) + state.velocity;
    }

    inline Quaternion getOrientation() const {
        return state.orientation;
    }

    inline V3D getAngularVelocity() const {
        return state.angularVelocity;
    }

    inline const RBState &getState() const {
        return state;
    }

    inline void setPosition(const P3D &pWorld) {
        state.pos = pWorld;
    }

    inline void setOrientation(const Quaternion &q) {
        state.orientation = q;
        state.orientation.normalize();
    }

    inline void setVelocity(const V3D &vWorld) {
        state.velocity = vWorld;
    }

    inline void setAngularVelocity(const V3D &wWorld) {
        state.angularVelocity = wWorld;
    }
};

}  // namespace crl::loco
