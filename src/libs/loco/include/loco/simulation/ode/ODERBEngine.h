#pragma once

#include <ode/ode.h>

#include "loco/robot/RBEngine.h"
#include "loco/robot/RBJoint.h"
#include "loco/robot/RBLoader.h"
#include "loco/simulation/ode/ODEContactForce.h"

#define MAX_CONTACT_FEEDBACK 200
#define MAX_AMOTOR_FEEDBACK 200

using namespace std;

namespace crl::loco {

// this structure is used to map a rigid body to the id of its ODE counterpart
typedef struct ODE_RB_Map_struct {
    dBodyID id;
    const std::shared_ptr<RB> rb;
    ODE_RB_Map_struct(dBodyID newId, const std::shared_ptr<RB> &newRb) : id(newId), rb(newRb) {}
} ODE_RB_Map;

// map joints that are controlled with desired position or velocity to an ODE motor
typedef struct ODE_MOTOR_JOINT_Map {
    dJointID motorID;
    const std::shared_ptr<RBJoint> joint;
    ODE_MOTOR_JOINT_Map(dJointID motorID, const std::shared_ptr<RBJoint> &joint) : motorID(motorID), joint(joint) {}
} ODE_Motor_Joint_Map;

// this structure is used to map joint to the id of its ODE counterpart
typedef struct ODE_Joint_Map_struct {
    dJointID id;
    const std::shared_ptr<RBJoint> j;
    ODE_Joint_Map_struct(dJointID newId, const std::shared_ptr<RBJoint> &newJ) : id(newId), j(newJ) {}
} ODE_Joint_Map;

// this structure is used to map a collision detection primitive to the id of its ODE counterpart
typedef struct ODE_CDP_Map_struct {
    dGeomID id;
    const std::shared_ptr<RB> rb;
    int cdpIndex;
    ODE_CDP_Map_struct(dGeomID newId, const std::shared_ptr<RB> &newRB, int _cdpIndex) : id(newId), rb(newRB), cdpIndex(_cdpIndex) {}
} ODE_CDP_Map;

/**
 * This class is used as a wrapper for the Open Dynamics Engine.
 */
class ODERBEngine : public RBEngine {
    friend void collisionCallBack(void *rbEngine, dGeomID o1, dGeomID o2);

public:
    /* simulation parameters */

    // if true, use ode quickstep() based on Gauss-Seidel
    // this tends to be faster and more stable but error prone.
    bool iterativeSolution = false;

    double contactDampingCoefficient = 0.00001;
    double contactStiffnessCoefficient = 0.2;

private:
    // ODE's id for the simulation world
    dWorldID worldID;
    // id of collision detection space
    dSpaceID spaceID;
    // id of contact group
    dJointGroupID contactGroupID;
    // keep track of the mapping between the rigid bodies and their ODE
    // counterparts with this
    DynamicArray<ODE_RB_Map> odeToRbs;
    DynamicArray<ODE_Joint_Map> odeToJoints;
    DynamicArray<ODE_CDP_Map> odeToCDP;
    DynamicArray<ODE_Motor_Joint_Map> motorToJointmap;

    // contact force
    DynamicArray<ODEContactForce> contactForces;
    dContact cps[10];

    dJointFeedback contactFeedback[MAX_CONTACT_FEEDBACK];
    dJointFeedback amotorFeedback[MAX_AMOTOR_FEEDBACK];
    // this is the current number of contact joints, for the current step of the
    // simulation
    int contactFeedbackCount = 0;

public:
    /**
     * default constructor
     */
    ODERBEngine();

    /**
     * destructor
     */
    ~ODERBEngine() override;

    void addRobotToEngine(const std::shared_ptr<Robot> &robot) override;

    void addRigidBodyToEngine(const std::shared_ptr<RB> &rb) override;

    void addJointToEngine(const std::shared_ptr<RBJoint> &j) override;

    /**
     * This method is used to integrate the simulation forward in time.
     */
    void step(double deltaT) override;

    /**
     * This method applies a force to a rigid body, at the specified point. The
     * point is specified in local coordinates, and the force is specified in
     * world coordinates.
     */
    void applyForceTo(const std::shared_ptr<RB> &b, const V3D &f, const P3D &p) override;

    /**
     * This method applies a torque to a rigid body. The torque is specified in
     * world coordinates.
     */
    void applyTorqueTo(const std::shared_ptr<RB> &b, const V3D &t);

    /**
     * this method applies a torque to a rigid body. The torque is specified in
     * relative body coordinates.
     */
    void applyRelativeTorqueTo(const std::shared_ptr<RB> &b, const V3D &t);

    /**
     * This method marks end effector contacted with ground, or other object.
     * For ODE, we use contact detection to populate contact state of EE.
     */
    void markEEContacts(double threshold = 0.01) override;

    void setGlobalCFM(double CFM);

    void setMotorsCFMAndFMax(double CFM, double FMAX);

#if 0
    /**
     * Update ODERBEngine using current rigidbodies.
     */
    void updateODERBEngineFromRBs();
#endif

    /**
     * Get the total contact force applied on a rigid body
     */
    virtual DynamicArray<ODEContactForce> getContactForceOnRB(const std::shared_ptr<RB> &b);

    // David G. added
    virtual dWorldID getWorldID();
    virtual dSpaceID getSpaceID();

    // Draw function for debugging purposes
    void drawODERB(const gui::Shader &shader);
    void drawODEJoint(const gui::Shader &shader);
    void drawODEContact(const gui::Shader &shader);

#if 0
    /**
     * This method is used to set the state of all the rigid body in the
     * physical world.
     */
    void setState(DynamicArray<double> *state, int start = 0);
#endif

    /**
     * set >0 for better stability. default value is 0.
     */
    void setGlobalDamping(double linear, double angular);

private:
    /**
     * This function calls ODE API for adding body to ODE world. It is called
     * when a rigid body as well as a robot is added to engine.
     */
    void addRigidBodyToODE(const std::shared_ptr<RB> &rb);

    /**
     * This function calls ODE API for adding joint to ODE world. It is called
     * when a rigid body as well as a robot is added to engine.
     */
    void addJointToODE(const std::shared_ptr<RBJoint> &j);

    /**
     * this method is used to set up an ODE sphere geom. It is properly placed
     * in body coordinates.
     */
    dGeomID getSphereGeom(const std::shared_ptr<const RRBCollisionSphere> &s);

    /**
     * this method is used to set up an ODE plane geom. It is properly placed in
     * body coordinates.
     */
    dGeomID getPlaneGeom(const std::shared_ptr<const RRBCollisionPlane> &p);

    /**
     *  this method is used to set up an ODE box geom. It is properly placed in
     * body coordinates.
     */
    dGeomID getBoxGeom(const std::shared_ptr<const RRBCollisionBox> &b);

    /**
     * this method is used to set up an ODE capsule geom. It is properly placed
     * in body coordinates. (axis along z-axis)
     */
    dGeomID getCapsuleGeom(const std::shared_ptr<const RRBCollisionCapsule> &c);

    /**
     * this method is used to set up an ODE cylinder geom It is properly placed
     * in body coordinates. (axis along z-axis)
     */
    dGeomID getCylinderGeom(const std::shared_ptr<const RRBCollisionCylinder> &c);

    /**
     * this method is used to process the collision between the two objects
     * passed in as parameters. More specifically, it is used to determine if
     * the collision should take place, and if so, it calls the method that
     * generates the contact points.
     */
    void processCollisions(dGeomID o1, dGeomID o2, DynamicArray<ODEContactForce> &contactForces);

    /**
     * this method is used to create ODE geoms for all the collision primitives
     * of the rigid body that is passed in as a paramter
     */
    void createODECollisionPrimitives(const std::shared_ptr<RB> &body);

    /**
     * this method is used to update ODE geoms for all the collision primitives
     * of the rigid body that is passed in as a paramter
     */
    void updateODECollisionPrimitives(const std::shared_ptr<RB> &body);

    /**
     * this method is used to copy the state of the ith rigid body to its ode
     * counterpart.
     */
    void setODEStateFromRB(int i);

    /**
     * this method is used to copy the state of the ith rigid body, from the
     * ode object to its rigid body counterpart
     */
    void setRBStateFromODE(int i);

    /**
     * This method is used to set up an ode hinge joint, based on the
     * information in the hinge joint passed in as a parameter
     */
    void setupODEHingeJoint(const std::shared_ptr<RBJoint> &hj);

    /**
     * Create an ODE motor (control purposes) for the specified joint
     */
    int getODEMotorForJoint(const std::shared_ptr<RBJoint> &j);

#if 0
    /**
     * This method is used to set up an ode universal joint, based on the
     * information in the universal joint passed in as a parameter
     */
    void setupODEUniversalJoint(UniversalJoint *uj);

    /**
     * This method is used to set up an ode ball-and-socket joint, based on the
     * information in the ball in socket joint passed in as a parameter
     */
    void setupODEBallAndSocketJoint(BallAndSocketJoint *basj);
#endif

    /**
     * this method is used to transfer the state of the rigid bodies, from the
     * simulator to the rigid body wrapper
     */
    void setRBStateFromEngine();

    /**
     * this method is used to transfer the state of the rigid bodies, from the
     * rigid body wrapper to the simulator's rigid bodies
     */
    void setEngineStateFromRB();
};

}  // namespace crl::loco
