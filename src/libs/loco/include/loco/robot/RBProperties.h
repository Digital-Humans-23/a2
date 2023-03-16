#pragma once

#include <crl-basic/gui/model.h>
#include <crl-basic/utils/mathUtils.h>

#include <memory>

namespace crl::loco {

/**
 * This class holds gui model (mesh)
 */
class RB3DModel : public crl::gui::Model {
public:
    RB3DModel() {}

    RB3DModel(const std::string& path) : path(path) {
        loadModel(path);
    }

    RigidTransformation localT;
    std::string description;
    std::string path;
    V3D color = V3D(0.9, 0.9, 0.9);
};

/**
 * Abstract class for collision primitives.
 */
class RRBCollsionShape {
public:
    virtual ~RRBCollsionShape() = 0;
};

/**
 * Collision primitive shaped a sphere.
 */
class RRBCollisionSphere : public RRBCollsionShape {
public:
    RRBCollisionSphere(const P3D& localCoordinates, double radius) : radius(radius), localCoordinates(localCoordinates) {}
    ~RRBCollisionSphere() override = default;

public:
    double radius;
    P3D localCoordinates;
};

/**
 * Collision primitive shaped 2D plane.
 */
class RRBCollisionPlane : public RRBCollsionShape {
public:
    RRBCollisionPlane(const P3D& p, const V3D& n) : p(p), n(n) {}
    ~RRBCollisionPlane() override = default;

public:
    P3D p = P3D(0, 0, 0);
    V3D n = V3D(0, 1, 0);
};

/**
 * Collision primitive shaped a box.
 */
class RRBCollisionBox : public RRBCollsionShape {
public:
    RRBCollisionBox(const P3D& localCoordinates, const Quaternion& localOrientation, const V3D& dimensions)
        : dimensions(dimensions), localCoordinates(localCoordinates), localOrientation(localOrientation) {}
    ~RRBCollisionBox() override = default;

public:
    P3D localCoordinates;
    Quaternion localOrientation;
    V3D dimensions;
};

/**
 * Collision primitive shaped a cylinder.
 */
class RRBCollisionCylinder : public RRBCollsionShape {
public:
    RRBCollisionCylinder(const P3D& localCoordinates, const Quaternion& localOrientation, double radius, double length)
        : length(length), radius(radius), localCoordinates(localCoordinates), localOrientation(localOrientation) {}
    ~RRBCollisionCylinder() override = default;

public:
    double radius, length;
    P3D localCoordinates;
    Quaternion localOrientation;
};

/**
 * Collision primitive shaped a capsule.
 */
class RRBCollisionCapsule : public RRBCollsionShape {
public:
    RRBCollisionCapsule(const P3D& localCoordinates, const Quaternion& localOrientation, double radius, double length)
        : length(length), radius(radius), localCoordinates(localCoordinates), localOrientation(localOrientation) {}
    ~RRBCollisionCapsule() override = default;

public:
    double radius, length;
    P3D localCoordinates;
    Quaternion localOrientation;
};

/**
 * End effector point. It's assumed to be a sphere with a certain radius.
 */
class RBEndEffector {
public:
    std::string name;
    double radius = 0.01;
    // center of ee in local frame
    P3D endEffectorOffset = P3D(0, 0, 0);
    // with ground, obstacle etc. (should be populated from physics engine or contact estimator)
    bool inContact = false;
    // contact force (should be populated from physics engine or contact force estimator)
    V3D contactForce = V3D(0, 0, 0);
};

/**
 * This class represents a container for the various properties of a rigid body
 */
class RBProperties {
public:
    // the mass
    double mass = 1.0;
    // we'll store the moment of inertia of the rigid body, in the local
    // coordinate frame
    Matrix3x3 MOI_local = Matrix3x3::Identity();

    // collision primitives that are relevant for this rigid body
    std::vector<std::shared_ptr<RRBCollsionShape>> collisionShapes;

    // meshes that are used to visualize the rigid body
    std::vector<RB3DModel> models;

    // end effector points
    std::vector<RBEndEffector> endEffectorPoints;

    // id of the rigid body
    int id = -1;

    // for selection via GUI
    bool selected = false;

    // for drawing abstract view
    double abstractViewCylRadius = 0.01;
    double endEffectorRadius = 0.01;
    V3D jointDrawColor = V3D(0.0, 0.0, 0.9);
    V3D highlightColor = V3D(1.0, 0.5, 0.5);
    V3D colSphereDrawColor = V3D(0.75, 0.0, 0.0);
    V3D endEffectorDrawColor = V3D(0.0, 1.0, 0.0);
    V3D contactedEndEffectorDrawColor = V3D(0.0, 1.0, 1.0);

    // draw color for rb primitive
    V3D color = V3D(0.5, 0.5, 0.5);

    // is this body is fixed to world
    bool fixed = false;

    // physics related coefficients
    double restitutionCoeff = 0;
    double frictionCoeff = 0.8;

public:
    /**
     * default constructor.
     */
    RBProperties() = default;

    /**
     * default destructor.
     */
    ~RBProperties() = default;

    /**
     * set the moment of inertia of the rigid body - symmetric 3x3 matrix, so
     * we need the six values for it.
     */
    void setMOI(double moi00, double moi11, double moi22, double moi01, double moi02, double moi12);

    /**
     * parallel axis theorem (translation)
     */
    void offsetMOI(double x, double y, double z);

    /**
     * similarity transformation (rotation)
     */
    void rotateMOI(double q, double x, double y, double z);

    Matrix3x3 getMOI(const Matrix3x3& RToWorld) const;

    Matrix3x3 getMOI(const Quaternion& q) const;
};

}  // namespace crl::loco
