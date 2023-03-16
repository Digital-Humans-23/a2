
#pragma once

#include <crl-basic/utils/mathDefs.h>

#include "loco/robot/RB.h"

namespace crl::loco {

/**
 * This class is mainly a container for a Contact Point. It holds information
 * such as the world coordinates of the contact point, the normal at the
 * contact, the rigid bodies that generated it, etc.
 */
class ODEContactForce {
public:
    // this is the world coordinate of the origin of the contact force...
    P3D cp;
    // this is the normal at the contact point
    V3D n;
    // and this is the penetration depth
    double d;
    // this is the first rigid body that participated in the contact
    RB *rb1;
    // and this is the second...
    RB *rb2;
    // and this is the force applied (with f being applied to rb1, and -f to rb2)
    V3D f;

    // provide a copy operator
    ODEContactForce &operator=(const ODEContactForce &other) {
        this->cp = other.cp;
        this->f = other.f;
        this->n = other.n;
        this->d = other.d;
        this->rb1 = other.rb1;
        this->rb2 = other.rb2;
        return *this;
    }

    ODEContactForce(RB *rb1, RB *rb2, const V3D &force, const V3D &normal, const P3D &contactPoint) {
        this->cp = contactPoint;
        this->f = force;
        this->n = normal;
        this->d = 0;
        this->rb1 = rb1;
        this->rb2 = rb2;
    }

    // provide a copy operator
    ODEContactForce(const ODEContactForce &other) {
        this->cp = other.cp;
        this->f = other.f;
        this->n = other.n;
        this->d = other.d;
        this->rb1 = other.rb1;
        this->rb2 = other.rb2;
    }

    // and a default constructor
    ODEContactForce() {
        rb1 = rb2 = NULL;
    }
};

}  // namespace crl::loco
