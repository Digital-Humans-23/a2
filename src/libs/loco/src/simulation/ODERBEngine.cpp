#include "loco/simulation/ode/ODERBEngine.h"

#include <memory>

#include "crl-basic/gui/renderer.h"
#include "crl-basic/utils/logger.h"

namespace crl::loco {

ODERBEngine::ODERBEngine() : RBEngine() {
    // Initialize the world, simulation space and joint groups
    dInitODE();
    worldID = dWorldCreate();
    spaceID = dHashSpaceCreate(0);
    contactGroupID = dJointGroupCreate(0);

    // make sure that when we destroy the space group, we destroy all the geoms inside it
    // see http://ode.org/wiki/index.php?title=Manual#Space_functions for more details.
    dSpaceSetCleanup(spaceID, 1);

    // set a few of the constants that ODE needs to be aware of
    dWorldSetContactSurfaceLayer(worldID, 0.001);    // the amount of interpenetration allowed between rbs
    dWorldSetContactMaxCorrectingVel(worldID, 1.0);  // maximum velocity that contacts are allowed to generate

    V3D gravity = RBGlobals::worldUp * RBGlobals::g;
    dWorldSetGravity(worldID, gravity[0], gravity[1], gravity[2]);
}

ODERBEngine::~ODERBEngine() {
    // destroy the ODE physical world, simulation space and joint group
    dJointGroupDestroy(contactGroupID);
    dSpaceDestroy(spaceID);
    dWorldDestroy(worldID);
    dCloseODE();
}

dGeomID ODERBEngine::getSphereGeom(const std::shared_ptr<const RRBCollisionSphere> &s) {
    dGeomID g = dCreateSphere(spaceID, s->radius);
    return g;
}

dGeomID ODERBEngine::getPlaneGeom(const std::shared_ptr<const RRBCollisionPlane> &p) {
    // and create the ground plane
    V3D n = p->n;
    V3D o = V3D(p->p);
    dGeomID g = dCreatePlane(spaceID, n[0], n[1], n[2], o.dot(n));
    return g;
}

dGeomID ODERBEngine::getBoxGeom(const std::shared_ptr<const RRBCollisionBox> &b) {
    dGeomID g = dCreateBox(spaceID, b->dimensions.x(), b->dimensions.y(), b->dimensions.z());
    return g;
}

dGeomID ODERBEngine::getCapsuleGeom(const std::shared_ptr<const RRBCollisionCapsule> &c) {
    dGeomID g = dCreateCapsule(spaceID, c->radius, c->length);
    return g;
}

dGeomID ODERBEngine::getCylinderGeom(const std::shared_ptr<const RRBCollisionCylinder> &c) {
    dGeomID g = dCreateCylinder(spaceID, c->radius, c->length);
    return g;
}

void ODERBEngine::processCollisions(dGeomID o1, dGeomID o2, DynamicArray<ODEContactForce> &contactForces) {
    dBodyID b1, b2;
    RB *rb1, *rb2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);
    rb1 = (RB *)dGeomGetData(o1);
    rb2 = (RB *)dGeomGetData(o2);

    bool joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);

    if (joined)
        return;
    if (!rb1->rbProps.fixed && !rb2->rbProps.fixed)
        return;
    //	if (rb1->rbProps.collisionGroupID == rb2->rbProps.collisionGroupID &&
    // rb1->rbProps.collisionGroupID != -1) 		return;

    // we'll use the minimum of the two coefficients of friction of the two
    // bodies.
    double mu1 = rb1->rbProps.frictionCoeff;
    double mu2 = rb2->rbProps.frictionCoeff;
    double mu_to_use = std::min(mu1, mu2);

    //	mu_to_use = 0;

    double eps1 = rb1->rbProps.restitutionCoeff;
    double eps2 = rb2->rbProps.restitutionCoeff;
    double eps_to_use = std::min(eps1, eps2);

    int maxContactCount = sizeof(cps) / sizeof(cps[0]);
    int num_contacts = dCollide(o1, o2, maxContactCount, &(cps[0].geom), sizeof(dContact));

    // and now add them contact points to the simulation
    for (int i = 0; i < num_contacts; i++) {
        // fill in the missing properties for the contact points
        cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
        cps[i].surface.mu = mu_to_use;

        cps[i].surface.bounce = eps_to_use;
        cps[i].surface.bounce_vel = 0.00001;

        cps[i].surface.soft_cfm = contactDampingCoefficient;
        cps[i].surface.soft_erp = contactStiffnessCoefficient;

        // create a joint, and link the two geometries.
        dJointID c = dJointCreateContact(worldID, contactGroupID, &cps[i]);
        dJointAttach(c, b1, b2);

        if (contactFeedbackCount >= MAX_CONTACT_FEEDBACK)
            Logger::consolePrint(
                "Warning: too many contacts are established. "
                "Some of them will not be reported.\n");
        else {
            if (contactForces.size() != (unsigned int)contactFeedbackCount) {
                Logger::consolePrint(
                    "Warning: Contact forces need to be cleared after each "
                    "simulation, otherwise the results are not predictable.\n");
            }
            contactForces.emplace_back();
            // now we'll set up the feedback for this contact joint
            contactForces[contactFeedbackCount].rb1 = rb1;
            contactForces[contactFeedbackCount].rb2 = rb2;
            contactForces[contactFeedbackCount].cp = P3D(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
            contactForces[contactFeedbackCount].n = V3D(cps[i].geom.normal[0], cps[i].geom.normal[1], cps[i].geom.normal[2]);
            dJointSetFeedback(c, &(contactFeedback[contactFeedbackCount]));
            contactFeedbackCount++;
        }
    }
}

void ODERBEngine::createODECollisionPrimitives(const std::shared_ptr<RB> &body) {
    // now we'll set up the body's collision detection primitives
    for (uint j = 0; j < body->rbProps.collisionShapes.size(); j++) {
        dGeomID g;
        if (const auto &s = std::dynamic_pointer_cast<RRBCollisionSphere>(body->rbProps.collisionShapes[j])) {
            g = getSphereGeom(s);
            dGeomSetData(g, body.get());
            dGeomSetBody(g, odeToRbs[body->rbProps.id].id);
            dGeomSetOffsetPosition(g, s->localCoordinates.x, s->localCoordinates.y, s->localCoordinates.z);
        } else if (const auto &p = std::dynamic_pointer_cast<RRBCollisionPlane>(body->rbProps.collisionShapes[j])) {
            g = getPlaneGeom(p);
            dGeomSetData(g, body.get());
        } else if (const auto &b = std::dynamic_pointer_cast<RRBCollisionBox>(body->rbProps.collisionShapes[j])) {
            dQuaternion offsetQ;
            offsetQ[0] = b->localOrientation.w();
            offsetQ[1] = b->localOrientation.x();
            offsetQ[2] = b->localOrientation.y();
            offsetQ[3] = b->localOrientation.z();
            g = getBoxGeom(b);
            dGeomSetData(g, body.get());
            dGeomSetBody(g, odeToRbs[body->rbProps.id].id);
            dGeomSetOffsetPosition(g, b->localCoordinates.x, b->localCoordinates.y, b->localCoordinates.z);
            dGeomSetOffsetQuaternion(g, offsetQ);
        } else if (const auto &cap = std::dynamic_pointer_cast<RRBCollisionCapsule>(body->rbProps.collisionShapes[j])) {
            dQuaternion offsetQ;
            offsetQ[0] = cap->localOrientation.w();
            offsetQ[1] = cap->localOrientation.x();
            offsetQ[2] = cap->localOrientation.y();
            offsetQ[3] = cap->localOrientation.z();
            g = getCapsuleGeom(cap);
            dGeomSetData(g, body.get());
            dGeomSetBody(g, odeToRbs[body->rbProps.id].id);
            dGeomSetOffsetPosition(g, cap->localCoordinates.x, cap->localCoordinates.y, cap->localCoordinates.z);
            dGeomSetOffsetQuaternion(g, offsetQ);
        } else if (const auto &cyl = std::dynamic_pointer_cast<RRBCollisionCylinder>(body->rbProps.collisionShapes[j])) {
            dQuaternion offsetQ;
            offsetQ[0] = cyl->localOrientation.w();
            offsetQ[1] = cyl->localOrientation.x();
            offsetQ[2] = cyl->localOrientation.y();
            offsetQ[3] = cyl->localOrientation.z();
            g = getCylinderGeom(cyl);
            dGeomSetData(g, body.get());
            dGeomSetBody(g, odeToRbs[body->rbProps.id].id);
            dGeomSetOffsetPosition(g, cyl->localCoordinates.x, cyl->localCoordinates.y, cyl->localCoordinates.z);
            dGeomSetOffsetQuaternion(g, offsetQ);
        } else {
            throwError("Unknown type of collision encountered...\n");
        }
        odeToCDP.emplace_back(g, body, j);
    }
}

void ODERBEngine::updateODECollisionPrimitives(const std::shared_ptr<RB> &body) {
    int nCdps = (int)body->rbProps.collisionShapes.size();
    for (int j = 0; j < nCdps; j++) {
        // find the geom that belongs to this CDP
        dGeomID g;

        bool found = false;
        for (int k = 0; k < (int)odeToCDP.size(); k++) {
            if (odeToCDP[k].rb == body && odeToCDP[k].cdpIndex == j) {
                g = odeToCDP[k].id;
                found = true;
                break;
            }
        }
        if (!found)
            continue;

#if 0
        if (CapsuleCDP *cdp = dynamic_cast<CapsuleCDP *>(body->cdps[j])) {
            setCapsuleGeomTransformation(cdp, g);
        }
        else if (BoxCDP *cdp = dynamic_cast<BoxCDP *>(body->cdps[j])) {
            dGeomBoxSetLengths(g, cdp->getXLen(), cdp->getYLen(),
                               cdp->getZLen());
            P3D c = cdp->getCenter();
            dGeomSetPosition(g, c[0], c[1], c[2]);
        }
        else {
            throwError("Unknown type of collision encountered...\n");
        }
#endif
        if (const auto &s = std::dynamic_pointer_cast<RRBCollisionSphere>(body->rbProps.collisionShapes[j])) {
            dGeomSphereSetRadius(g, s->radius);
            dGeomSetOffsetPosition(g, s->localCoordinates.x, s->localCoordinates.y, s->localCoordinates.z);
        } else if (const auto &p = std::dynamic_pointer_cast<RRBCollisionPlane>(body->rbProps.collisionShapes[j])) {
            V3D n = p->n;
            V3D o = V3D(p->p);
            dGeomPlaneSetParams(g, n[0], n[1], n[2], o.dot(n));
        } else {
            throwError("Unknown type of collision encountered...\n");
        }
    }
}

void ODERBEngine::setODEStateFromRB(int i) {
    if (i < 0 || (uint)i >= odeToRbs.size())
        return;

    const auto &body = odeToRbs[i].rb;

    P3D bpos = body->getWorldCoordinates(P3D());
    dBodySetPosition(odeToRbs[i].id, bpos.x, bpos.y, bpos.z);

    Quaternion bq = body->getOrientation();
    dQuaternion tempQ;
    tempQ[0] = bq.w();
    tempQ[1] = bq.x();
    tempQ[2] = bq.y();
    tempQ[3] = bq.z();
    dBodySetQuaternion(odeToRbs[i].id, tempQ);

    V3D bvel = body->getVelocityForPoint_local(P3D());
    dBodySetLinearVel(odeToRbs[i].id, bvel.x(), bvel.y(), bvel.z());

    V3D bw = body->getAngularVelocity();
    dBodySetAngularVel(odeToRbs[i].id, bw.x(), bw.y(), bw.z());
}

void ODERBEngine::setRBStateFromODE(int i) {
    const dReal *tempData;

    tempData = dBodyGetPosition(odeToRbs[i].id);
    odeToRbs[i].rb->setPosition(P3D(tempData[0], tempData[1], tempData[2]));

    tempData = dBodyGetQuaternion(odeToRbs[i].id);
    odeToRbs[i].rb->setOrientation(Quaternion(tempData[0], tempData[1], tempData[2], tempData[3]));

    tempData = dBodyGetLinearVel(odeToRbs[i].id);
    odeToRbs[i].rb->setVelocity(V3D(tempData[0], tempData[1], tempData[2]));

    tempData = dBodyGetAngularVel(odeToRbs[i].id);
    odeToRbs[i].rb->setAngularVelocity(V3D(tempData[0], tempData[1], tempData[2]));
}

int ODERBEngine::getODEMotorForJoint(const std::shared_ptr<RBJoint> &j) {
    int motorID = -1;
    // create a motor if desired...
    if (j) {
        dJointID aMotor = nullptr;
        // Retrieve motor from map
        for (uint m = 0; m < motorToJointmap.size(); m++) {
            if (motorToJointmap[m].joint == j) {
                aMotor = motorToJointmap[m].motorID;
                motorID = (int)m;
                break;
            }
        }
        if (aMotor == nullptr) {
            aMotor = dJointCreateAMotor(worldID, 0);
            motorToJointmap.emplace_back(aMotor, j);
            motorID = (int)motorToJointmap.size() - 1;
            dJointAttach(aMotor, odeToRbs[(int)(j->parent->rbProps.id)].id, odeToRbs[(int)(j->child->rbProps.id)].id);
            dJointSetAMotorMode(aMotor, dAMotorUser);
            dJointSetAMotorNumAxes(aMotor, 1);
            dJointSetAMotorParam(aMotor, dParamFMax, j->maxTorque);
            // just in case the configuration of the joint changes (i.e.
            // rotation axis), update the motor axis
            V3D a = j->parent->getWorldCoordinates(j->rotationAxis);
            dJointSetAMotorAxis(aMotor, 0, 1, a[0], a[1], a[2]);
            // we set ERP and CFM in step
            // dJointSetAMotorParam(aMotor, dParamCFM,
            // hj->motorConstraintForceRegularizer);
        }
    } else
        Logger::consolePrint("Joint motors are only implemented for Hinge Joints for now!\n");

    return motorID;
}

void ODERBEngine::setupODEHingeJoint(const std::shared_ptr<RBJoint> &hj) {
    dJointID j;
    uint k = 0;
    for (k = 0; k < odeToJoints.size(); k++) {
        if (odeToJoints[k].j == hj) {
            j = odeToJoints[k].id;
            break;
        }
    }

    if (k == odeToJoints.size()) {
        j = dJointCreateHinge(worldID, 0);
        odeToJoints.emplace_back(j, hj);
        dJointAttach(j, odeToRbs[(int)(hj->child->rbProps.id)].id, odeToRbs[(int)(hj->parent->rbProps.id)].id);
    }

    P3D p = hj->child->getWorldCoordinates(hj->cJPos);
    dJointSetHingeAnchor(j, p[0], p[1], p[2]);
    V3D a = hj->parent->getWorldCoordinates(hj->rotationAxis);
    dJointSetHingeAxis(j, a[0], a[1], a[2]);

    // now set the joint limits
    if (hj->jointAngleLimitsActive) {
        dJointSetHingeParam(j, dParamLoStop, hj->minAngle);
        dJointSetHingeParam(j, dParamHiStop, hj->maxAngle);
    }
}

#if 0
void ODERBEngine::setupODEUniversalJoint(UniversalJoint *uj)
{
    dJointID j;
    uint k = 0;
    for (; k < odeToJoints.size(); k++) {
        if (odeToJoints[k].j == uj) {
            j = odeToJoints[k].id;
            break;
        }
    }

    if (k == odeToJoints.size()) {
        j = dJointCreateUniversal(worldID, 0);
        odeToJoints.push_back(ODE_Joint_Map_struct(j, uj));
        dJointAttach(j, odeToRbs[(int)(uj->child->id)].id,
                     odeToRbs[(int)(uj->parent->id)].id);
    }

    P3D p = uj->child->getWorldCoordinates(uj->cJPos);
    dJointSetUniversalAnchor(j, p[0], p[1], p[2]);

    V3D a = uj->parent->getWorldCoordinates(uj->rotAxisParent);
    V3D b = uj->child->getWorldCoordinates(uj->rotAxisChild);

    dJointSetUniversalAxis1(j, b[0], b[1], b[2]);
    dJointSetUniversalAxis2(j, a[0], a[1], a[2]);

    // now set the joint limits
    if (uj->shouldUseJointLimits() == false)
        return;

    dJointSetUniversalParam(j, dParamLoStop2, uj->minAnglePRA);
    dJointSetUniversalParam(j, dParamHiStop2, uj->maxAnglePRA);
    dJointSetUniversalParam(j, dParamLoStop, uj->minAngleCRA);
    dJointSetUniversalParam(j, dParamHiStop, uj->maxAngleCRA);
}
#endif

#if 0
void ODERBEngine::setupODEBallAndSocketJoint(BallAndSocketJoint *basj)
{
    dJointID j;
    uint k = 0;
    for (; k < odeToJoints.size(); k++) {
        if (odeToJoints[k].j == basj) {
            j = odeToJoints[k].id;
            break;
        }
    }
    if (k == odeToJoints.size()) {
        j = dJointCreateBall(worldID, 0);
        odeToJoints.push_back(ODE_Joint_Map_struct(j, basj));
        dJointAttach(j, odeToRbs[(int)(basj->child->id)].id,
                     odeToRbs[(int)(basj->parent->id)].id);
    }

    P3D p = basj->child->getWorldCoordinates(basj->cJPos);
    // now we'll set the world position of the ball-and-socket joint. It is
    // important that the bodies are placed in the world properly at this point
    dJointSetBallAnchor(j, p[0], p[1], p[2]);

    // now deal with the joint limits
    if (basj->shouldUseJointLimits() == true) {
        Logger::consolePrint("Joint limits for ball and socket joints are not "
                             "yet implemented!\n");
    }
}
#endif

void ODERBEngine::setRBStateFromEngine() {
    // now update given rigid bodies...
    for (int i = 0; i < this->odeToRbs.size(); i++) {
        setRBStateFromODE(i);
    }
}

void ODERBEngine::setEngineStateFromRB() {
    // now update given rigid bodies...
    for (int i = 0; i < this->odeToRbs.size(); i++) {
        setODEStateFromRB(i);
    }
}

void ODERBEngine::addRobotToEngine(const std::shared_ptr<Robot> &robot) {
    RBEngine::addRobotToEngine(robot);

    // ode joint limit should be added in zero configuration
    RobotState backup(*robot);
    robot->setZeroState();

    // now add every rb and joint in robot to ode world
    for (int i = 0; i < robot->getRigidBodyCount(); i++) {
        addRigidBodyToODE(robot->getRigidBody(i));
    }
    for (int i = 0; i < robot->getJointCount(); i++) {
        addJointToODE(robot->getJoint(i));
    }

    // back to current state
    robot->setState(backup);
}

void ODERBEngine::addRigidBodyToEngine(const std::shared_ptr<RB> &rb) {
    RBEngine::addRigidBodyToEngine(rb);
    addRigidBodyToODE(rb);
}

void ODERBEngine::addRigidBodyToODE(const std::shared_ptr<RB> &rb) {
    // create and link rigid body to ODE corresponding body
    dBodyID newBody = dBodyCreate(worldID);
    odeToRbs.emplace_back(newBody, rb);
    // the ID of this rigid body will be its index in the
    rb->rbProps.id = (int)odeToRbs.size() - 1;
    // we will use the user data of the object to store the index in this
    // mapping as well, for easy retrieval
    dBodySetData(odeToRbs[rb->rbProps.id].id, rb.get());
    // PROCESS THE COLLISION PRIMITIVES OF THE BODY
    createODECollisionPrimitives(rb);

    // if the body is fixed, we'll create constraints to keep it in place...
    if (rb->rbProps.fixed) {
        dBodySetKinematic(newBody);
    } else {
        // SET THE INERTIAL PARAMETERS
        dMass m;

        // set the mass and principal moments of inertia for this object
        m.setZero();

        m.setParameters(rb->rbProps.mass, 0, 0, 0, rb->rbProps.MOI_local(0, 0), rb->rbProps.MOI_local(1, 1), rb->rbProps.MOI_local(2, 2),
                        rb->rbProps.MOI_local(0, 1), rb->rbProps.MOI_local(0, 2), rb->rbProps.MOI_local(1, 2));

        dBodySetMass(odeToRbs[rb->rbProps.id].id, &m);
    }

    setODEStateFromRB(rb->rbProps.id);
}

void ODERBEngine::addJointToEngine(const std::shared_ptr<RBJoint> &j) {
    RBEngine::addJointToEngine(j);
    addJointToODE(j);
}

void ODERBEngine::addJointToODE(const std::shared_ptr<RBJoint> &j) {
    // update the ODE state for the parent and child rigid bodies, since the
    // joint axis/position will be set up in world coordinates
    setODEStateFromRB(j->parent->rbProps.id);
    setODEStateFromRB(j->child->rbProps.id);

    // connect the joint to the two bodies
#if 0
    if (BallAndSocketJoint *bJoint = dynamic_cast<BallAndSocketJoint *>(j)) {
        setupODEBallAndSocketJoint(bJoint);
    }
    else if (HingeJoint *hJoint = dynamic_cast<HingeJoint *>(j)) {
        setupODEHingeJoint(hJoint);
    }
    else if (UniversalJoint *uJoint = dynamic_cast<UniversalJoint *>(j)) {
        setupODEUniversalJoint(uJoint);
    }
    else
        throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints "
                   "are currently supported.\n");
#endif
    // at the moment we only support hinge joint
    setupODEHingeJoint(j);
}

#if 0
void ODERBEngine::updateODERBEngineFromRBs()
{
    // now we'll make sure that the joint constraints are satisfied
    for (uint i = 0; i < rbs.size(); i++) {

        dBodyID odeBody = NULL;
        for (uint k = 0; k < odeToRbs.size(); k++) {
            if (odeToRbs[k].rb == rbs[i]) {
                odeBody = odeToRbs[k].id;
                break;
            }
        }
        if (odeBody == NULL)
            throwError("RobotRB not found...\n");

        // PROCESS THE COLLISION PRIMITIVES OF THE BODY
        updateODECollisionPrimitives(rbs[i]);

        // reset the inertial parameters, in case they've changed...
        if (rbs[i]->rbProps.fixed == false) {
            dMass m;

            // set the mass and principal moments of inertia for this object
            m.setZero();
            m.setParameters(odeToRbs[i].rb->rbProps.mass, 0, 0, 0,
                            odeToRbs[i].rb->rbProps.MOI_local(0, 0),
                            odeToRbs[i].rb->rbProps.MOI_local(1, 1),
                            odeToRbs[i].rb->rbProps.MOI_local(2, 2),
                            odeToRbs[i].rb->rbProps.MOI_local(0, 1),
                            odeToRbs[i].rb->rbProps.MOI_local(0, 2),
                            odeToRbs[i].rb->rbProps.MOI_local(1, 2));

            dBodySetMass(odeToRbs[i].id, &m);
            setODEStateFromRB(i);
        }
    }

    // now we will go through all the joints, and update them (setup...() does
    // update them if they exist already)
    for (uint i = 0; i < joints.size(); i++) {
#if 0
        // connect the joint to the two bodies
        if (BallAndSocketJoint *bJoint =
                dynamic_cast<BallAndSocketJoint *>(joints[i])) {
            setupODEBallAndSocketJoint(bJoint);
        }
        else if (HingeJoint *hJoint = dynamic_cast<HingeJoint *>(joints[i])) {
            setupODEHingeJoint(hJoint);
        }
        else if (UniversalJoint *uJoint =
                     dynamic_cast<UniversalJoint *>(joints[i])) {
            setupODEUniversalJoint(uJoint);
        }
        else
            throwError("Ooops.... Only BallAndSocket, Hinge and Universal "
                       "joints are currently supported.\n");
#endif
        HingeJoint *hJoint = joints[i];
        setupODEHingeJoint(hJoint);
    }
}
#endif

#if 0
void ODERBEngine::setState(DynamicArray<double> *state, int start)
{
    AbstractRBEngine::setState(state, start);
}
#endif

void collisionCallBack(void *rbEngine, dGeomID o1, dGeomID o2) {
    ODERBEngine *pWorld = (ODERBEngine *)rbEngine;
    pWorld->processCollisions(o1, o2, pWorld->contactForces);
}

void ODERBEngine::setGlobalCFM(double CFM) {
    dWorldSetCFM(worldID, CFM);
}

void ODERBEngine::setMotorsCFMAndFMax(double CFM, double FMAX) {
    for (const auto &odeToJoint : odeToJoints) {
        const auto &hj = odeToJoint.j;

        dJointID aMotor;
        bool found = false;
        // Retrieve motor from map
        for (uint m = 0; m < motorToJointmap.size(); m++) {
            if (motorToJointmap[m].joint == hj) {
                aMotor = motorToJointmap[m].motorID;
                found = true;
            }
        }
        if (!found)
            continue;

        dJointSetAMotorParam(aMotor, dParamCFM, CFM);
        dJointSetAMotorParam(aMotor, dParamFMax, FMAX);
    }
}

void ODERBEngine::step(double deltaT) {
    // make sure that the state of the RB's is synchronized with the engine...
    setEngineStateFromRB();

    // restart the counter for the joint feedback terms
    contactFeedbackCount = 0;

    // apply control inputs as needed...
    for (uint j = 0; j < odeToJoints.size(); j++) {
        if (odeToJoints[j].j->controlMode == RBJointControlMode::POSITION_MODE || odeToJoints[j].j->controlMode == RBJointControlMode::VELOCITY_MODE) {
            int motorID = getODEMotorForJoint(odeToJoints[j].j);
            if (motorID >= 0) {
                // the motor may have been placed in limbo, so reattach it to
                // its rigid bodies...
                dJointAttach(motorToJointmap[motorID].motorID, odeToRbs[(int)(odeToJoints[j].j->parent->rbProps.id)].id,
                             odeToRbs[(int)(odeToJoints[j].j->child->rbProps.id)].id);

                if (odeToJoints[j].j->controlMode == RBJointControlMode::POSITION_MODE) {
                    V3D rotAxis = odeToJoints[j].j->parent->getWorldCoordinates(odeToJoints[j].j->rotationAxis);
                    rotAxis.normalize();
                    if (rotAxis.norm() < 0.9)
                        rotAxis = V3D(1, 0, 0);

                    double currentAngle = odeToJoints[j].j->getCurrentJointAngle();
                    double desiredAngle = motorToJointmap[motorID].joint->desiredControlPosition;
                    double rotAngle = desiredAngle - currentAngle;

                    // ERP = h*kp / (h*kp + kd)
                    double erp = deltaT * odeToJoints[j].j->motorKp / (deltaT * odeToJoints[j].j->motorKp + odeToJoints[j].j->motorKd);

                    if (rotAngle >= 2 * PI)
                        rotAngle -= 2 * PI;
                    if (rotAngle <= -2 * PI)
                        rotAngle += 2 * PI;

                    double desSpeed = -rotAngle / deltaT * erp;
                    boundToRange(&desSpeed, -odeToJoints[j].j->maxSpeed, odeToJoints[j].j->maxSpeed);
                    // Logger::consolePrint("joint: %d - desired speed: %lf (%
                    // lf "
                    //                      "% lf % lf % lf)\n",
                    //                      j, desSpeed, qErr.s, qErr.v[0],
                    //                      qErr.v[1], qErr.v[2]);
                    // Logger::consolePrint("joint: %d\t%lf\n", j, desSpeed);

                    // CFM = 1 / (h*kp + kd)
                    dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamCFM, 1.0 / (deltaT * odeToJoints[j].j->motorKp + odeToJoints[j].j->motorKd));
                    dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
                    dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, desSpeed);
                } else {
                    // the desired relative angular velocity needs to be
                    // expressed in world coordinates...
                    V3D rotAxis = odeToJoints[j].j->parent->getWorldCoordinates(odeToJoints[j].j->rotationAxis);
                    rotAxis.normalize();
                    if (rotAxis.norm() < 0.9)
                        rotAxis = V3D(1, 0, 0);
                    double angVelocity = odeToJoints[j].j->desiredControlSpeed;
                    dJointSetAMotorNumAxes(motorToJointmap[motorID].motorID, 1);
                    dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
                    dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, -angVelocity);
                }
                // setup the feedback structure so that we can read off the
                // torques that were applied
                if (j < MAX_AMOTOR_FEEDBACK)
                    dJointSetFeedback(motorToJointmap[motorID].motorID, &(amotorFeedback[j]));
            } else
                Logger::consolePrint(
                    "Warning: control mode for joint %d requires a motor, but "
                    "none has been created...\n");
        }

        if (odeToJoints[j].j->controlMode == RBJointControlMode::PASSIVE_MODE) {
            int motorID = getODEMotorForJoint(odeToJoints[j].j);
            // we need to place the motor in limbo, such that it does not
            // interfere with torque controller...
            if (motorID >= 0)
                dJointAttach(motorToJointmap[motorID].motorID, 0, 0);
        }

        if (odeToJoints[j].j->controlMode == RBJointControlMode::FORCE_MODE) {
            int motorID = getODEMotorForJoint(odeToJoints[j].j);
            // we need to place the motor in limbo, such that it does not
            // interfere with torque controller...
            if (motorID >= 0)
                dJointAttach(motorToJointmap[motorID].motorID, 0, 0);

            V3D rotAxis = odeToJoints[j].j->parent->getWorldCoordinates(odeToJoints[j].j->rotationAxis.normalized());
            V3D t = rotAxis * odeToJoints[j].j->desiredControlTorque;

            // if there's force limit specified, we cut off torque applied to joint
            if (odeToJoints[j].j->jointTorqueLimitActive && abs(odeToJoints[j].j->desiredControlTorque) > odeToJoints[j].j->maxTorque) {
                t = rotAxis * odeToJoints[j].j->maxTorque;
            }

            // we will apply to the parent a positive torque, and to the child a
            // negative torque
            dBodyAddTorque(odeToRbs[odeToJoints[j].j->parent->rbProps.id].id, -t[0], -t[1], -t[2]);
            dBodyAddTorque(odeToRbs[odeToJoints[j].j->child->rbProps.id].id, t[0], t[1], t[2]);
        }
    }

    // get an up-to-date list of contact forces...
    contactForces.clear();
    dJointGroupEmpty(contactGroupID);
    // initiate the collision detection
    dSpaceCollide(spaceID, this, &collisionCallBack);

    // advance the simulation
    if (iterativeSolution == false)
        dWorldStep(worldID, deltaT);
    else
        dWorldQuickStep(worldID, deltaT);

    // copy over the state of the ODE bodies to the rigid bodies...
    setRBStateFromEngine();

    // copy over the torque information...
    for (uint j = 0; j < odeToJoints.size(); j++) {
        if (odeToJoints[j].j->controlMode != RBJointControlMode::POSITION_MODE && odeToJoints[j].j->controlMode != RBJointControlMode::VELOCITY_MODE)
            continue;
        int motorID = getODEMotorForJoint(odeToJoints[j].j);

        if (motorID >= 0) {
            // If this is a hinge joint with a motor attached, then we've used
            // the motors, so see what torque they ended up applying...
            V3D rotAxis = odeToJoints[j].j->parent->getWorldCoordinates(odeToJoints[j].j->rotationAxis.normalized());
            if (j < MAX_AMOTOR_FEEDBACK)
                odeToJoints[j].j->motorFeedback = V3D(amotorFeedback[j].t1[0], amotorFeedback[j].t1[1], amotorFeedback[j].t1[2]).dot(rotAxis);
        }
    }

    // copy over the force information for the contact forces
    for (int i = 0; i < contactFeedbackCount; i++) {
        contactForces[i].f = V3D(contactFeedback[i].f1[0], contactFeedback[i].f1[1], contactFeedback[i].f1[2]);
        // make sure that the force always points away from the static rbs
        if (contactForces[i].rb1->rbProps.fixed && !contactForces[i].rb2->rbProps.fixed) {
            contactForces[i].n = contactForces[i].n * (-1);
            contactForces[i].f = contactForces[i].f * (-1);
            const auto &tmpBdy = contactForces[i].rb1;
            contactForces[i].rb1 = contactForces[i].rb2;
            contactForces[i].rb2 = tmpBdy;
        }
    }

    // finally check over every ee for contacts with ground
    markEEContacts(0.001);
}

void ODERBEngine::applyForceTo(const std::shared_ptr<RB> &b, const V3D &f, const P3D &p) {
    if (!b)
        return;
    dBodyAddForceAtRelPos(odeToRbs[b->rbProps.id].id, f.x(), f.y(), f.z(), p.x, p.y, p.z);
}

void ODERBEngine::applyTorqueTo(const std::shared_ptr<RB> &b, const V3D &t) {
    if (!b)
        return;
    dBodyAddTorque(odeToRbs[b->rbProps.id].id, t.x(), t.y(), t.z());
}

void ODERBEngine::applyRelativeTorqueTo(const std::shared_ptr<RB> &b, const V3D &t) {
    if (!b)
        return;
    dBodyAddRelTorque(odeToRbs[b->rbProps.id].id, t.x(), t.y(), t.z());
}

DynamicArray<ODEContactForce> ODERBEngine::getContactForceOnRB(const std::shared_ptr<RB> &bs) {
    DynamicArray<ODEContactForce> grfs;

    for (auto it = contactForces.begin(); it != contactForces.end(); ++it)
        if (((*it).rb1->rbProps.id == bs->rbProps.id) || ((*it).rb2->rbProps.id == bs->rbProps.id)) {
            grfs.push_back(*it);
        }

    return grfs;
}

dWorldID ODERBEngine::getWorldID() {
    return worldID;
};

dSpaceID ODERBEngine::getSpaceID() {
    return spaceID;
};

void ODERBEngine::drawODERB(const gui::Shader &shader) {
    for (uint i = 0; i < this->odeToRbs.size(); i++) {
        if (this->odeToRbs[i].rb->rbProps.fixed)
            continue;

        const dReal *tempData;
        tempData = dBodyGetPosition(odeToRbs[i].id);
        P3D pos(tempData[0], tempData[1], tempData[2]);
        tempData = dBodyGetQuaternion(odeToRbs[i].id);
        Quaternion rbQ(tempData[0], tempData[1], tempData[2], tempData[3]);

        dMass dmass;
        dBodyGetMass(odeToRbs[i].id, &dmass);

        double mass = dmass.mass;

        Matrix3x3 inertia;
        inertia << dmass.I[0], dmass.I[1], dmass.I[2], dmass.I[4], dmass.I[5], dmass.I[6], dmass.I[8], dmass.I[9], dmass.I[10];

        Eigen::EigenSolver<Matrix3x3> eigenvalueSolver(inertia);

        Eigen::Vector3cd principleMomentsOfInertia = eigenvalueSolver.eigenvalues();

        assert(IS_ZERO(principleMomentsOfInertia[0].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()));

        Eigen::Matrix3cd V = eigenvalueSolver.eigenvectors();

        double Ixx = principleMomentsOfInertia[0].real();  // = m(y2 + z2)/12
        double Iyy = principleMomentsOfInertia[1].real();  // = m(z2 + x2)/12
        double Izz = principleMomentsOfInertia[2].real();  // = m(y2 + x2)/12

        double x = sqrt((Iyy + Izz - Ixx) * 6 / mass);
        double y = sqrt((Izz + Ixx - Iyy) * 6 / mass);
        double z = sqrt((Ixx + Iyy - Izz) * 6 / mass);

        P3D pmin(-x / 2, -y / 2, -z / 2), pmax(x / 2, y / 2, z / 2);

        if (V.determinant().real() < 0.0) {
            V(0, 2) *= -1;
            V(1, 2) *= -1;
            V(2, 2) *= -1;
        }
        assert(IS_ZERO(abs(V.determinant().real() - 1.0)) && "Rotation matrices have a determinant which is equal to 1.0!");

        Quaternion q(V.real());
        drawCuboid(pos, rbQ * q, V3D(x, y, z), shader, V3D(1.0, 1.0, 0.0));
    }
}

void ODERBEngine::drawODEJoint(const gui::Shader &shader) {
    // Hinge joints
    for (uint i = 0; i < joints.size(); i++) {
        dVector3 tempData;
        dJointGetHingeAnchor(odeToJoints[i].id, tempData);
        P3D p(tempData[0], tempData[1], tempData[2]);
        dJointGetHingeAxis(odeToJoints[i].id, tempData);
        V3D v(tempData[0], tempData[1], tempData[2]);
        drawArrow3d(p, v * 0.1, 0.01, shader, V3D(0.0, 0.0, 1.0));
    }
}

void ODERBEngine::drawODEContact(const gui::Shader &shader) {
    for (const auto &c : contactForces) {
        drawArrow3d(c.cp, c.f * 0.01, 0.01, shader, V3D(0.0, 1.0, 0.0));
    }
}

void ODERBEngine::markEEContacts(double threshold) {
    // TODO: this is kinda messy...
    for (const auto &odeToRb : this->odeToRbs) {
        const auto &rb = odeToRb.rb;

        // update ee tip as well as mark contact
        for (auto &ee : rb->rbProps.endEffectorPoints) {
            ee.inContact = false;
            ee.contactForce = V3D(0, 0, 0);

            for (auto &c : contactForces) {
                // find contacts on rb
                if ((c.rb1->rbProps.id == rb->rbProps.id) || (c.rb2->rbProps.id == rb->rbProps.id)) {
                    // and check if contact is in certain distance from EE center
                    P3D contactInLocal = rb->getLocalCoordinates(c.cp);
                    if (V3D(ee.endEffectorOffset, contactInLocal).norm() <= ee.radius + threshold) {
                        ee.inContact = true;
                        ee.contactForce = c.rb1->rbProps.id == rb->rbProps.id ? c.f : -c.f;
                    }
                }
            }
        }
    }
}

void ODERBEngine::setGlobalDamping(double linear, double angular) {
    dWorldSetDamping(worldID, linear, angular);
}

}  // namespace crl::loco
