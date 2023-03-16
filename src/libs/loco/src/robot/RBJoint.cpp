#include "loco/robot/RBJoint.h"

namespace crl::loco {

P3D RBJoint::getWorldPosition() const {
    return (child->getWorldCoordinates(cJPos) + parent->getWorldCoordinates(pJPos)) / 2.0;
}

Quaternion RBJoint::computeRelativeOrientation() const {
    // if qp is the quaternion that gives the orientation of the parent, and qc
    // gives the orientation of the child, then  qp^-1 * qc gives the relative
    // orientation between the child and the parent (child to parent)
    return (parent->getOrientation().inverse() * child->getOrientation()).normalized();
}

void RBJoint::fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities) {
    assert(child && parent);

    // first fix the relative orientation
    if (fixOrientations) {
        Quaternion qRel = computeRelativeOrientation();
        // make sure that the relative rotation between the child and the parent
        // is around the a axis
        V3D axis = qRel.vec().normalized();
        // this is the rotation angle around the axis above, which may not be
        // the rotation axis
        double rotAngle = getRotationAngle(qRel, axis);
        // get the rotation angle around the correct axis now (we are not in the
        // world frame now)
        double ang = axis.dot(rotationAxis) * rotAngle;
        // compute the correct child orientation
        child->setOrientation(parent->getOrientation() * getRotationQuaternion(ang, rotationAxis));
    }

    // now worry about the joint positions
    if (fixPositions) {
        // compute the vector rc from the child's joint position to the child's
        // center of mass (in rbEngine coordinates)
        V3D rc = child->getWorldCoordinates(V3D(cJPos, P3D(0, 0, 0)));
        // and the vector rp that represents the same quanity but for the parent
        V3D rp = parent->getWorldCoordinates(V3D(pJPos, P3D(0, 0, 0)));

        // the location of the child's CM is now: pCM - rp + rc
        child->setPosition(parent->getWorldCoordinates(P3D()) + (rc - rp));
    }

    if (fixAngularVelocities) {
        V3D wRel = child->getAngularVelocity() - parent->getAngularVelocity();
        V3D worldRotAxis = parent->getWorldCoordinates(rotationAxis);
        // only keep the part that is aligned with the rotation axis...
        child->setAngularVelocity(parent->getAngularVelocity() + worldRotAxis * wRel.dot(worldRotAxis));
    }

    // fix the velocities, if need be
    if (fixLinVelocities) {
        // we want to get the relative velocity at the joint to be 0. This can
        // be accomplished in many different ways, but this approach only
        // changes the linear velocity of the child
        V3D pJPosVel = parent->getVelocityForPoint_local(pJPos);
        V3D cJPosVel = child->getVelocityForPoint_local(cJPos);
        child->setVelocity(child->getVelocityForPoint_local(P3D()) - (cJPosVel - pJPosVel));
        V3D testVel = parent->getVelocityForPoint_local(pJPos) - child->getVelocityForPoint_local(cJPos);
        assert(IS_ZERO(testVel.norm()));
    }
}

double RBJoint::getCurrentJointAngle() const {
    return getRotationAngle(computeRelativeOrientation(), rotationAxis);
}

}  // namespace crl::loco
