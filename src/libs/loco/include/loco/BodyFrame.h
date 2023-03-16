#pragma once

#include "loco/robot/GeneralizedCoordinatesRobotRepresentation.h"
#include "loco/robot/Robot.h"
#include "loco/utils/MathUtils.h"

namespace crl::loco {

// A body frame is defined only through the position and heading (rotation about
// the vertical axis), as well as their rates of change
class BodyFrame {
public:
    // this is the position, in world, of the body frame
    P3D p = P3D(0, 0, 0);
    // this is the heading, indicating the direction in which the robot is
    // facing
    double h = 0;
    // the velocity of the body frame's origin, in world coordinates
    V3D v = V3D(0, 0, 0);
    // the rate of change of the heading
    double omega = 0;
    // the acceleration of the body frame's origin, again in world coordinates
    V3D a = V3D(0, 0, 0);
    // and the rate of change of omega, the angular acceleration for the heading
    double alpha = 0;

    BodyFrame(GCRR* rr, const V3D& a = V3D(0, 0, 0), double alpha = 0) {
        this->p = P3D(rr->getQVal(0), rr->getQVal(1), rr->getQVal(2));
        this->h = rr->getQVal(3);
        this->v = V3D(rr->getQDotVal(0), rr->getQDotVal(1), rr->getQDotVal(2));
        this->omega = rr->getQDotVal(3);
        this->a = a;
        this->alpha = alpha;
    }

    BodyFrame(const P3D& p, const Quaternion& q) {
        this->p = p;
        this->h = computeHeadingFromQuaternion(q);
    }

    BodyFrame(const P3D& p, double h, const V3D& v = V3D(0, 0, 0), double omega = 0, const V3D& a = V3D(0, 0, 0), double alpha = 0) {
        this->p = p;
        this->h = h;
        this->v = v;
        this->omega = omega;
        this->a = a;
        this->alpha = alpha;
    }

    // the resulting vector points from the origin of the body frame to pW. Both
    // pW and the result are in world coordinates
    V3D getOffsetVectorToPoint(const P3D& pW) const {
        return V3D(p, pW);
    }

    //orientation/heading takes vectors from local coords of the body frame to world coords
    P3D getLocalCoordinatesFor(const P3D& pW) const {
        return P3D() + getRotationQuaternion(-h, RBGlobals::worldUp) * getOffsetVectorToPoint(pW);
    }

    //orientation/heading takes vectors from local coords of the body frame to world coords
    V3D getLocalCoordinatesFor(const V3D& vW) const {
        return getRotationQuaternion(-h, RBGlobals::worldUp) * vW;
    }

    P3D getWorldCoordinatesFor(const P3D& pL) const {
        return p + getRotationQuaternion(h, RBGlobals::worldUp) * V3D(P3D(0, 0, 0), pL);
    }

    V3D getWorldCoordinatesFor(const V3D& vL) {
        return getRotationQuaternion(h, RBGlobals::worldUp) * vL;
    }

    // returns the velocity of point pW (specified in world coordinates) due to
    // the motion of the body frame (i.e. if pW was rigidly attached to the body
    // frame). the resulting velocity vector is represented in world
    // coordinates.
    V3D getVelocityForPoint(const P3D& pW) {
        V3D r = getOffsetVectorToPoint(pW);
        V3D Omega = RBGlobals::worldUp * omega;
        return v + Omega.cross(r);
    }

    // returns the acceleration of point pW (specified in world coordinates) due
    // to the motion of the body frame (i.e. if pW was rigidly attached to the
    // body frame). the resulting acceleration vector is represented in world
    // coordinates.
    V3D getAccelerationForPoint(const P3D& pW) {
        V3D r = getOffsetVectorToPoint(pW);
        V3D Omega = RBGlobals::worldUp * omega;
        V3D Alpha = RBGlobals::worldUp * alpha;
        return a + Omega.cross(Omega.cross(r)) + Alpha.cross(r);
    }
};
}  // namespace crl::loco
