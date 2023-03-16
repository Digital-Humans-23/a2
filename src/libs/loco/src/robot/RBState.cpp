//
// Created by Dongho Kang on 08.12.21.
//

#include "loco/robot/RBState.h"

namespace crl::loco {

RBState::RBState(const RBState &other) {
    this->pos = other.pos;
    this->orientation = other.orientation;
    this->velocity = other.velocity;
    this->angularVelocity = other.angularVelocity;
}

RBState &RBState::operator=(const RBState &other) {
    this->pos = other.pos;
    this->orientation = other.orientation;
    this->velocity = other.velocity;
    this->angularVelocity = other.angularVelocity;
    return *this;
}

bool RBState::operator==(const RBState &other) const {
    if (V3D(pos, other.pos).norm() > 1e-10)
        return false;
    //q and -q represent the same rotation...
    if (!sameRotation(orientation, other.orientation))
        return false;
    if ((velocity - other.velocity).norm() > 1e-10)
        return false;
    if ((angularVelocity - other.angularVelocity).norm() > 1e-10)
        return false;
    return true;
}

}  // namespace crl::loco