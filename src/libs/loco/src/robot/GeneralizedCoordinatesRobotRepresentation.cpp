#include "loco/robot/GeneralizedCoordinatesRobotRepresentation.h"

#include "crl-basic/utils/utils.h"

namespace crl::loco {

// TODO: for this implementation, we have chosen not to precompute any
// quantities (e.g. world coords of joints or joint axes, position/orientations
// of RBs, etc), trading speed in favor of a conceptually clean implementation.
// If ever the need will arise, things like RB orientations, positions and
// rotation axes of joints, etc can be precomputed as soon as the q's get set,
// which presumably will make things faster, but it will also make this class
// more difficult to work with as things would always need to be kept in sync
// properly...

GeneralizedCoordinatesRobotRepresentation::GeneralizedCoordinatesRobotRepresentation(const std::shared_ptr<Robot> &a) {
    robot = a;
    setupGeneralizedCoordinatesStructure();
    syncGeneralizedCoordinatesWithRobotState();
}

// sets up the whole structure of the robot - a tree structure
void GeneralizedCoordinatesRobotRepresentation::setupGeneralizedCoordinatesStructure() {
    int nTotalDim = 6;  // at the very least, the dofs for the root of the robot
    for (int i = 0; i < 6; i++) {
        qParentIndex.push_back(i - 1);
        jointIndexForQ.push_back(-1);
    }

    for (uint i = 0; i < robot->jointList.size(); i++) {
        int nDim = 1;  // Only hinge joints!

        jointCoordStartIndex.push_back(nTotalDim);
        jointCoordsDimSize.push_back(nDim);
        for (int j = 0; j < nDim; j++)
            jointIndexForQ.push_back(i);

        nTotalDim += nDim;

        // the child can be connected straight to the root...
        if (robot->jointList[i]->parent->pJoint == nullptr) {
            // the hierarchichal parent for this joint is the last root rotation
            // DOF
            qParentIndex.push_back(5);
        } else {
            // this, going backwards through the transformation hierarchy, is
            // the index of the first q of the parent...
            int qpIndex = robot->jointList[i]->parent->pJoint->jIndex;
            // check where the parent joint's q DOFs start -- that's the
            // hierarchichal parent for the last DOF of the current joint
            qParentIndex.push_back(jointCoordStartIndex[qpIndex] + jointCoordsDimSize[qpIndex] - 1);
        }

        for (int j = 1; j < nDim; j++)
            qParentIndex.push_back(jointCoordStartIndex[i] + j - 1);
    }

    resize(q, nTotalDim);
    resize(qDot, nTotalDim);
}

// returns the qIndex at which this joint starts
int GeneralizedCoordinatesRobotRepresentation::getQIndexForJoint(const std::shared_ptr<const RBJoint> &joint) const {
    return jointCoordStartIndex[joint->jIndex];
}

void GeneralizedCoordinatesRobotRepresentation::integrateGenerlizedAccelerationsForwardInTime(const dVector &a, double dt) {
    setQDot(qDot + a * dt);
    setQ(q + qDot * dt);
}

// updates q and qDot given current state of robot
void GeneralizedCoordinatesRobotRepresentation::syncGeneralizedCoordinatesWithRobotState() {
    // write out the position of the root...
    RobotState state(*robot);
    P3D pos = state.getPosition();
    Quaternion orientation = state.getOrientation();

    q[0] = pos.x;
    q[1] = pos.y;
    q[2] = pos.z;

    // Root
    computeEulerAnglesFromQuaternion(orientation, getQAxis(5), getQAxis(4), getQAxis(3), q[5], q[4], q[3]);

    // Now go through each joint, and decompose it as appropriate...
    for (uint i = 0; i < robot->jointList.size(); i++) {
        // Only hinge joints!
        computeRotationAngleFromQuaternion(state.getJointRelativeOrientation(i), getQAxis(jointCoordStartIndex[i] + 0), q[jointCoordStartIndex[i] + 0]);

        //		Quaternion Qerr =
        // getRotationQuaternion(q[jointCoordStartIndex[i]],
        // getQAxis(jointCoordStartIndex[i])).inverse() *
        // state.getJointRelativeOrientation(i); 		if
        //(Qerr.angularDistance(Quaternion::Identity()) > 1e-8)
        // printf("joint %d: err %15.15lf\n", i,
        // Qerr.angularDistance(Quaternion::Identity()));
    }

    // now update the velocities qDot
    std::vector<V3D> worldRelJointVelocities;
    for (uint i = 0; i < robot->jointList.size(); i++)
        // the reduced state stores angular velocities in parent coords, so
        // change them to world
        worldRelJointVelocities.push_back(getOrientationFor(robot->jointList[i]->parent) * (state.getJointRelativeAngVelocity(i)));

    projectWorldCoordsValuesIntoGeneralizedSpace(state.getVelocity(), state.getAngularVelocity(), worldRelJointVelocities, qDot);
}

// returns the local coord vector from the parent of q(qIndex) to q(qIndex)
V3D GeneralizedCoordinatesRobotRepresentation::getOffsetFromParentToQ(int qIndex) const {
    int qIndexParent = qParentIndex[qIndex];
    // if they both belong to the same joint, then their locations coincide
    if (jointIndexForQ[qIndex] == -1 || jointIndexForQ[qIndex] == jointIndexForQ[qIndexParent])
        return V3D(0, 0, 0);
    return V3D(getPivotPointLocalPosition(robot->jointList[jointIndexForQ[qIndex]]->parent), robot->jointList[jointIndexForQ[qIndex]]->pJPos);
}

// returns the axis corresponding to the indexed generalized coordinate,
// expressed in local coordinates
V3D GeneralizedCoordinatesRobotRepresentation::getQAxis(int qIndex) const {
    if (qIndex >= 0 || qIndex < 6) {
        // the first three are the translational dofs of the body
        if (qIndex == 0)
            return V3D(1, 0, 0);
        if (qIndex == 1)
            return V3D(0, 1, 0);
        if (qIndex == 2)
            return V3D(0, 0, 1);
        if (qIndex == 3)
            return RBGlobals::worldUp;  // y - yaw
        if (qIndex == 4)
            return RBGlobals::worldUp.cross(robot->forward);  // x - pitch
        if (qIndex == 5)
            return robot->forward;  // z - roll
    }

    int jIndex = jointIndexForQ[qIndex];

    // Only hinge joint!
    return robot->jointList[jIndex]->rotationAxis;
}

// returns the local position of the point that rb pivots about (i.e. location
// of the parent joint), in coordinate frame of rb
P3D GeneralizedCoordinatesRobotRepresentation::getPivotPointLocalPosition(const std::shared_ptr<const RB> &rb) const {
    if (rb->pJoint == nullptr)
        return P3D(0, 0, 0);

    return rb->pJoint->cJPos;
}

void GeneralizedCoordinatesRobotRepresentation::syncRobotStateWithGeneralizedCoordinates() const {
    RobotState rs(*robot);
    getReducedRobotState(rs);
    robot->setState(rs);
}

// given the current state of the generalized representation, output the reduced
// state of the robot
void GeneralizedCoordinatesRobotRepresentation::getReducedRobotState(RobotState &state) const {
    // set the position, velocity, rotation and angular velocity for the root
    state.setPosition(P3D(0, 0, 0) + getQAxis(0) * q[0] + getQAxis(1) * q[1] + getQAxis(2) * q[2]);
    state.setVelocity(V3D(getQAxis(0) * qDot[0] + getQAxis(1) * qDot[1] + getQAxis(2) * qDot[2]));
    state.setAngularVelocity(V3D(getWorldCoordsAxisForQ(3) * qDot[3] + getWorldCoordsAxisForQ(4) * qDot[4] + getWorldCoordsAxisForQ(5) * qDot[5]));
    state.setOrientation(getOrientationFor(robot->root));

    for (uint i = 0; i < robot->jointList.size(); i++) {
        Quaternion jointOrientation = Quaternion::Identity();
        V3D jointAngularVel;
        for (int j = 0; j < jointCoordsDimSize[i]; j++) {
            int jIndex = jointCoordStartIndex[i] + j;
            jointOrientation *= getRotationQuaternion(q[jIndex], getQAxis(jIndex));
            jointAngularVel += getWorldCoordsAxisForQ(jIndex) * qDot[jIndex];
        }
        // the relative angular velocity is now in world coords, so we need to
        // change it in the coords of the parent
        //		printf("joint %d Q: %lf %lf %lf %lf\n", i,
        // jointOrientation.w(), jointOrientation.x(), jointOrientation.y(),
        // jointOrientation.z()); 		double tmpVal = 0;
        //		computeRotationAngleFromQuaternion(jointOrientation,
        // getQAxis(jointCoordStartIndex[i] + 0), tmpVal);
        // printf("error in the reconstruction: %15.15lf\n",
        // q[jointCoordStartIndex[i] + 0] - tmpVal);

        state.setJointRelativeOrientation(jointOrientation, i);
        state.setJointRelativeAngVelocity(getOrientationFor(robot->jointList[i]->parent).inverse() * (jointAngularVel), i);
    }
    // and done...
}

// sets the current q values
void GeneralizedCoordinatesRobotRepresentation::setQ(const dVector &qNew) {
    assert(q.size() == qNew.size());
    // NOTE: we don't update the angular velocities. The assumption is that the
    // correct behavior is that the joint relative angular velocities don't
    // change, although the world relative values of the rotations do
    q = qNew;
}

// gets the current q values
void GeneralizedCoordinatesRobotRepresentation::getQ(dVector &q_copy) const {
    q_copy = q;
}

void GeneralizedCoordinatesRobotRepresentation::getQAndQDotFromReducedState(const RobotState &rs, dVector &q_copy, dVector &qDot_copy) {
    dVector q_old = q;
    dVector qDot_old = qDot;

    RobotState oldState(*robot);
    robot->setState(rs);
    syncGeneralizedCoordinatesWithRobotState();
    getQ(q_copy);
    getQDot(qDot_copy);
    robot->setState(oldState);

    setQ(q_old);
    setQDot(qDot);
}

void GeneralizedCoordinatesRobotRepresentation::getQFromReducedState(const RobotState &rs, dVector &q_copy) {
    dVector q_old = q;
    dVector qDot_old = qDot;

    RobotState oldState(*robot);
    robot->setState(rs);
    syncGeneralizedCoordinatesWithRobotState();
    getQ(q_copy);
    robot->setState(oldState);

    setQ(q_old);
    setQDot(qDot);
}

// sets the current qDot values
void GeneralizedCoordinatesRobotRepresentation::setQDot(const dVector &qDotNew) {
    assert(q.size() == qDotNew.size());
    qDot = qDotNew;
}

// gets the current qDot values
void GeneralizedCoordinatesRobotRepresentation::getQDot(dVector &qDot_copy) const {
    qDot_copy = qDot;
}

void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D &vec, const V3D &a, const V3D &b, const V3D &c, double &aVal,
                                                                                     double &bVal, double &cVal) {
    // we cannot assume the vectors form an orthogonal basis, so we have to
    // solve a system to compute the unknowns aVal, bVal, cVal. We assume that
    // vec, a, b and c are all specified in the same coordinate frame
    Matrix3x3 m, mInv;
    m(0, 0) = a[0];
    m(0, 1) = b[0];
    m(0, 2) = c[0];
    m(1, 0) = a[1];
    m(1, 1) = b[1];
    m(1, 2) = c[1];
    m(2, 0) = a[2];
    m(2, 1) = b[2];
    m(2, 2) = c[2];

    mInv = m.inverse();

    V3D sol = V3D(mInv * vec);

    aVal = sol[0];
    bVal = sol[1];
    cVal = sol[2];

    // check the solution...
    V3D res = V3D(m * sol) - vec;
    if (res.norm() > 0.0001)
        std::cout << "Failed to properly compute generalized coordinates for "
                     "the input :("
                  << std::endl;
}

void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D &vec, const V3D &a, const V3D &b, double &aVal, double &bVal) {
    // we assume the two vectors are orthogonal to each other...
    assert(IS_ZERO(a.dot(b)));

    // in this setting, projecting the vector on the generalized coord axes is
    // just a matter of taking dot products...
    aVal = vec.dot(a);
    bVal = vec.dot(b);
}

void GeneralizedCoordinatesRobotRepresentation::projectVectorOnGeneralizedCoordsAxes(const V3D &vec, const V3D &a, double &aVal) {
    aVal = vec.dot(a);
}

void GeneralizedCoordinatesRobotRepresentation::projectWorldCoordsValuesIntoGeneralizedSpace(const V3D &linearVal, const V3D &angularVal,
                                                                                             const std::vector<V3D> &jointVal,
                                                                                             dVector &generalizedCoordinates) {
    resize(generalizedCoordinates, q.size());
    projectVectorOnGeneralizedCoordsAxes(linearVal, getQAxis(0), getQAxis(1), getQAxis(2), generalizedCoordinates[0], generalizedCoordinates[1],
                                         generalizedCoordinates[2]);
    projectVectorOnGeneralizedCoordsAxes(angularVal, getWorldCoordsAxisForQ(3), getWorldCoordsAxisForQ(4), getWorldCoordsAxisForQ(5), generalizedCoordinates[3],
                                         generalizedCoordinates[4], generalizedCoordinates[5]);

    // now go through each joint, and decompose it as appropriate...
    for (uint i = 0; i < robot->jointList.size(); i++) {
        int startIndex = jointCoordStartIndex[i];
        // Only hinge joint!
        projectVectorOnGeneralizedCoordsAxes(jointVal[i], getWorldCoordsAxisForQ(startIndex + 0), generalizedCoordinates[startIndex + 0]);
    }
}

// returns the world coordinates for point p, which is specified in the local
// coordinates of rb (relative to its COM): p(q)
P3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinates(const P3D &p, const std::shared_ptr<const RB> &rb) const {
    V3D offset(p);
    int qIndex = getQStartIndexForRB(rb);

    if (rb->pJoint != nullptr)
        offset = V3D(rb->pJoint->cJPos, p);

    // 2 here is the index of the first translational DOF of the root
    while (qIndex > 2) {
        V3D qAxis = getQAxis(qIndex);
        V3D offsetRot = rotateVec(offset, q[qIndex], qAxis);
        V3D offsetFromParent = getOffsetFromParentToQ(qIndex);
        offset = offsetFromParent + offsetRot;
        qIndex = qParentIndex[qIndex];
    }

    return P3D() + V3D(getQAxis(0) * q[0] + getQAxis(1) * q[1] + getQAxis(2) * q[2] + offset);
}

// returns the world coordinates for vector b, which is specified in the local
// coordinates of rb: v(q)
V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinates(const V3D &v, const std::shared_ptr<const RB> &rb) const {
    V3D theVector(v);
    int qIndex = getQStartIndexForRB(rb);

    // 2 here is the index of the first translational DOF of the root
    while (qIndex > 2) {
        V3D qAxis = getQAxis(qIndex);
        theVector = rotateVec(theVector, q[qIndex], qAxis);
        qIndex = qParentIndex[qIndex];
    }

    return theVector;
}

// returns the velocity (world coordinates) of the point p, which is specified
// in the local coordinates of rb (relative to its COM). I.e. p(q)
V3D GeneralizedCoordinatesRobotRepresentation::getVelocityFor(const P3D &p, const std::shared_ptr<const RB> &rb) const {
    // pDot(q) = dp/dq * qDot
    Matrix dpdq;
    dVector res;

    compute_dpdq(p, rb, dpdq);
    res = dpdq * qDot;

    return V3D(res[0], res[1], res[2]);
}

V3D GeneralizedCoordinatesRobotRepresentation::getAngularVelocityFor(const std::shared_ptr<const RB> &rb) const {
    // w(q) = dR/dq * qDot
    Matrix dRdq;
    dVector res;

    compute_angular_jacobian(rb, dRdq);
    res = dRdq * qDot;

    return V3D(res[0], res[1], res[2]);
}

// returns the global orientation associated with a specific dof q...
Quaternion GeneralizedCoordinatesRobotRepresentation::getWorldRotationForQ(int qIndex) const {
    Quaternion qRes = Quaternion::Identity();
    // 2 here is the index of the first translational DOF of the root -- these
    // dofs do not contribute to the orientation of the rigid bodies...
    while (qIndex > 2) {
        qRes = getRelOrientationForQ(qIndex) * qRes;
        qIndex = qParentIndex[qIndex];
    }
    return qRes;
}

Quaternion GeneralizedCoordinatesRobotRepresentation::getRelOrientationForQ(int qIndex) const {
    if (qIndex < 3)
        return Quaternion::Identity();
    return getRotationQuaternion(q[qIndex], getQAxis(qIndex));
}

// this is a somewhat slow function to use if we must iterate through multiple
// rigid bodies...
V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordsAxisForQ(int qIndex) const {
    if (qIndex < 3)
        return getQAxis(qIndex);
    return getWorldRotationForQ(qIndex) * getQAxis(qIndex);
}

// returns the world-relative orientation for rb
Quaternion GeneralizedCoordinatesRobotRepresentation::getOrientationFor(const std::shared_ptr<const RB> &rb) const {
    //	return Quaternion();
    /*
            if (rb->pJoint == nullptr)
                    return getWorldRotationForQ(5);

            int jIndex = rb->pJoint->jIndex;

            int startIndex = jointCoordStartIndex[jIndex];

            int jDim = jointCoordsDimSize[jIndex];

            return getWorldRotationForQ(startIndex + jDim - 1);
    */
    int qIndex = getQStartIndexForRB(rb);

    return getWorldRotationForQ(qIndex);
}

// computes the jacobian dp/dq that tells you how the world coordinates of p
// change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq) const {
    resize(dpdq, 3, q.size());
    dpdq.fill(0);

    // todo: this bit of code (and other jacobians/higher order derivatives) can
    // probably be made faster if ever they become the bottleneck... seems like
    // it would be easy enough to precompute orientations/joint
    // positions/offsets in a first pass from the bottom up, instead of always
    // recomputing them for each j... after all, we know that dpdq_i is the
    // cross product of the vector from pos of q_i to p (all in world coords)
    // with the rotation axis of q_i (in world coords)...

    int startIndex = getQStartIndexForRB(rb);

    int loopIndex = startIndex;
    // 2 here is the index of the first translational DOF of the root
    while (loopIndex > 2) {
        V3D offset(p);
        if (rb->pJoint != nullptr)
            offset = V3D(rb->pJoint->cJPos, p);
        int qIndex = startIndex;

        // compute the offset from the location of joint qIndex to point p - if
        // these quantitites were known in world coordinates, then it would be
        // clear we wouldn't need the loop
        while (qIndex > loopIndex) {
            offset = getOffsetFromParentToQ(qIndex) + rotateVec(offset, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        // this is how the vector computed above changes with the rotation angle
        offset = getQAxis(qIndex).cross(offset);

        // and now bring this vector to world coordinates, since the step above
        // was computed in the coordinate frame of joint q. again, if RB
        // orientations were precomputed, we could just read it off now...
        while (qIndex > 2) {
            offset = rotateVec(offset, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        for (int i = 0; i < 3; i++)
            dpdq(i, loopIndex) = offset[i];

        loopIndex = qParentIndex[loopIndex];
    }

    dpdq(0, 0) = 1;
    dpdq(1, 1) = 1;
    dpdq(2, 2) = 1;
}

// computes the jacobian dp/dq that tells you how the world coordinates of p
// change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dvdq(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &dvdq) const {
    resize(dvdq, 3, q.size());
    dvdq.fill(0);

    int startIndex = getQStartIndexForRB(rb);

    int loopIndex = startIndex;
    // 2 here is the index of the first translational DOF of the root
    while (loopIndex > 2) {
        V3D theVector(v);
        int qIndex = startIndex;

        while (qIndex > loopIndex) {
            theVector = rotateVec(theVector, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        theVector = getQAxis(qIndex).cross(theVector);

        while (qIndex > 2) {
            theVector = rotateVec(theVector, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        for (int i = 0; i < 3; i++)
            dvdq(i, loopIndex) = theVector[i];

        loopIndex = qParentIndex[loopIndex];
    }
}

// estimates the linear jacobian dp/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq) {
    resize(dpdq, 3, (int)q.size());

    for (int i = 0; i < q.size(); i++) {
        double val = q[i];
        double h = 0.0001;

        q[i] = val + h;
        P3D p_p = getWorldCoordinates(p, rb);

        q[i] = val - h;
        P3D p_m = getWorldCoordinates(p, rb);

        q[i] = val;
        V3D dpdq_i = V3D(p_m, p_p) / (2 * h);
        dpdq(0, i) = dpdq_i[0];
        dpdq(1, i) = dpdq_i[1];
        dpdq(2, i) = dpdq_i[2];
    }
}

// estimates the jacobian dv/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &dvdq) {
    resize(dvdq, 3, (int)q.size());

    for (int i = 0; i < q.size(); i++) {
        double val = q[i];
        double h = 0.0001;

        q[i] = val + h;
        V3D p_p = getWorldCoordinates(v, rb);

        q[i] = val - h;
        V3D p_m = getWorldCoordinates(v, rb);

        q[i] = val;
        V3D dvdq_i = (p_p - p_m) / (2 * h);
        dvdq(0, i) = dvdq_i[0];
        dvdq(1, i) = dvdq_i[1];
        dvdq(2, i) = dvdq_i[2];
    }
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian(const P3D &p, const std::shared_ptr<const RB> &rb) {
    Matrix dpdq_analytic, dpdq_estimated;
    compute_dpdq(p, rb, dpdq_analytic);
    estimate_linear_jacobian(p, rb, dpdq_estimated);

    //	print("../out/dpdq_analytic.mat", dpdq_analytic);
    //	print("../out/dpdq_estimated.mat", dpdq_estimated);

    bool error = false;

    for (int i = 0; i < dpdq_analytic.rows(); i++)
        for (int j = 0; j < dpdq_analytic.cols(); j++) {
            double err = dpdq_analytic(i, j) - dpdq_estimated(i, j);
            if (fabs(err) > 0.0001) {
                std::cout << "Error at: " << i << " " << j << ": analytic: " << dpdq_analytic(i, j) << " estimated " << dpdq_estimated(i, j)
                          << " error: " << err << std::endl;
                error = true;
            }
        }

    return !error;
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian(const V3D &v, const std::shared_ptr<const RB> &rb) {
    Matrix dvdq_analytic, dvdq_estimated;
    compute_dvdq(v, rb, dvdq_analytic);
    estimate_linear_jacobian(v, rb, dvdq_estimated);

    //	print("../out/dpdq_analytic.mat", dpdq_analytic);
    //	print("../out/dpdq_estimated.mat", dpdq_estimated);

    bool error = false;

    for (int i = 0; i < dvdq_analytic.rows(); i++)
        for (int j = 0; j < dvdq_analytic.cols(); j++) {
            double err = dvdq_analytic(i, j) - dvdq_estimated(i, j);
            if (fabs(err) > 0.0001) {
                std::cout << "Error at: " << i << " " << j << ": analytic: " << dvdq_analytic(i, j) << " estimated " << dvdq_estimated(i, j)
                          << " error: " << err << std::endl;
                error = true;
            }
        }

    return !error;
}

// returns the index of the last (e.g. highest) dof that controls the
// position/orientation of the rb
int GeneralizedCoordinatesRobotRepresentation::getQStartIndexForRB(const std::shared_ptr<const RB> &rb) const {
    int startIndex = 5;
    if (rb->pJoint != nullptr)
        startIndex = jointCoordStartIndex[rb->pJoint->jIndex] + jointCoordsDimSize[rb->pJoint->jIndex] - 1;
    return startIndex;
}

// computes the matrix that tells you how the jacobian dp/dq changes with
// respect to q_i. Returns true if it contains non-zero elements, false
// otherwise
bool GeneralizedCoordinatesRobotRepresentation::compute_ddpdq_dqi(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &ddpdq_dqi, int q_i) const {
    resize(ddpdq_dqi, 3, (int)q.size());

    int startIndex = getQStartIndexForRB(rb);

    // if q_i is not one of the ancestors of rb, then it means the jacobian dpdq
    // does not depent on it, so check first...
    int qIndex = startIndex;
    bool isAncestor = false;
    while (qIndex > 2) {
        if (q_i == qIndex) {
            isAncestor = true;
            break;
        }
        qIndex = qParentIndex[qIndex];
    }

    if (!isAncestor)
        return false;

    // input is valid, so we must compute this derivative...
    int loopIndex = startIndex;
    // 2 here is the index of the first translational DOF of the root
    while (loopIndex > 2) {
        V3D offset(p);
        if (rb->pJoint != nullptr)
            offset = V3D(rb->pJoint->cJPos, p);
        int qIndex = startIndex;

        int stopIndex = loopIndex;
        if (q_i > loopIndex)
            stopIndex = q_i;

        while (qIndex > stopIndex) {
            offset = getOffsetFromParentToQ(qIndex) + rotateVec(offset, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        while (qIndex > 2) {
            if (qIndex == loopIndex)
                offset = getQAxis(qIndex).cross(offset);
            if (qIndex == q_i)
                offset = getQAxis(qIndex).cross(offset);

            offset = rotateVec(offset, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        for (int i = 0; i < 3; i++)
            ddpdq_dqi(i, loopIndex) = offset[i];

        loopIndex = qParentIndex[loopIndex];
    }

    return true;
}

// computes the matrix that tells you how the jacobian dv/dq changes with
// respect to q_i. Returns true if it contains non-zero elements, false
// otherwise
bool GeneralizedCoordinatesRobotRepresentation::compute_ddvdq_dqi(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &ddvdq_dqi, int q_i) const {
    resize(ddvdq_dqi, 3, (int)q.size());

    int startIndex = getQStartIndexForRB(rb);

    // if q_i is not one of the ancestors of rb, then it means the jacobian dpdq
    // does not depent on it, so check first...
    int qIndex = startIndex;
    bool isAncestor = false;
    while (qIndex > 2) {
        if (q_i == qIndex) {
            isAncestor = true;
            break;
        }
        qIndex = qParentIndex[qIndex];
    }

    if (!isAncestor)
        return false;

    // input is valid, so we must compute this derivative...
    int loopIndex = startIndex;
    // 2 here is the index of the first translational DOF of the root
    while (loopIndex > 2) {
        V3D theVector(v);
        int qIndex = startIndex;

        int stopIndex = loopIndex;
        if (q_i > loopIndex)
            stopIndex = q_i;

        while (qIndex > stopIndex) {
            theVector = rotateVec(theVector, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        while (qIndex > 2) {
            if (qIndex == loopIndex)
                theVector = getQAxis(qIndex).cross(theVector);
            if (qIndex == q_i)
                theVector = getQAxis(qIndex).cross(theVector);

            theVector = rotateVec(theVector, q[qIndex], getQAxis(qIndex));
            qIndex = qParentIndex[qIndex];
        }

        for (int i = 0; i < 3; i++)
            ddvdq_dqi(i, loopIndex) = theVector[i];

        loopIndex = qParentIndex[loopIndex];
    }

    return true;
}

// estimates the change of dp/dq with respect to q_i
void GeneralizedCoordinatesRobotRepresentation::estimate_ddpdq_dqi(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &ddpdq_dqi, int q_i) {
    resize(ddpdq_dqi, 3, (int)q.size());
    Matrix dpdq_p, dpdq_m;
    dpdq_p = ddpdq_dqi;
    dpdq_m = ddpdq_dqi;

    double val = q[q_i];
    double h = 0.0001;

    q[q_i] = val + h;
    compute_dpdq(p, rb, dpdq_p);

    q[q_i] = val - h;
    compute_dpdq(p, rb, dpdq_m);

    q[q_i] = val;

    ddpdq_dqi = (dpdq_p - dpdq_m) / (2 * h);
}

// estimates the change of dv/dq with respect to q_i
void GeneralizedCoordinatesRobotRepresentation::estimate_ddvdq_dqi(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &ddvdq_dqi, int q_i) {
    resize(ddvdq_dqi, 3, (int)q.size());
    Matrix dvdq_p, dvdq_m;
    dvdq_p = ddvdq_dqi;
    dvdq_m = ddvdq_dqi;

    double val = q[q_i];
    double h = 0.0001;

    q[q_i] = val + h;
    compute_dvdq(v, rb, dvdq_p);

    q[q_i] = val - h;
    compute_dvdq(v, rb, dvdq_m);

    q[q_i] = val;

    ddvdq_dqi = (dvdq_p - dvdq_m) / (2 * h);
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian_derivatives(const P3D &p, const std::shared_ptr<const RB> &rb) {
    Matrix dpdq_analytic, dpdq_estimated;

    bool error = false;

    for (int k = 0; k < getDimensionSize(); k++) {
        compute_ddpdq_dqi(p, rb, dpdq_analytic, k);
        estimate_ddpdq_dqi(p, rb, dpdq_estimated, k);

        //		dpdq_analytic.printMatrix("out\\dpdq.mat");

        for (int i = 0; i < dpdq_analytic.rows(); i++)
            for (int j = 0; j < dpdq_analytic.cols(); j++) {
                double err = dpdq_analytic(i, j) - dpdq_estimated(i, j);
                if (fabs(err) > 0.0001) {
                    std::cout << "Error when computing ddpdq_dq " << k << " at: " << i << " " << j << ": analytic: " << dpdq_analytic(i, j) << " estimated "
                              << dpdq_estimated(i, j) << " error: " << err << std::endl;
                    error = true;
                }
            }
    }

    return !error;
}

bool GeneralizedCoordinatesRobotRepresentation::test_linear_jacobian_derivatives(const V3D &v, const std::shared_ptr<const RB> &rb) {
    Matrix dvdq_analytic, dvdq_estimated;

    bool error = false;

    for (int k = 0; k < getDimensionSize(); k++) {
        compute_ddvdq_dqi(v, rb, dvdq_analytic, k);
        estimate_ddvdq_dqi(v, rb, dvdq_estimated, k);

        //		dpdq_analytic.printMatrix("out\\dpdq.mat");

        for (int i = 0; i < dvdq_analytic.rows(); i++)
            for (int j = 0; j < dvdq_analytic.cols(); j++) {
                double err = dvdq_analytic(i, j) - dvdq_estimated(i, j);
                if (fabs(err) > 0.0001) {
                    std::cout << "Error when computing ddvdq_dq " << k << " at: " << i << " " << j << ": analytic: " << dvdq_analytic(i, j) << " estimated "
                              << dvdq_estimated(i, j) << " error: " << err << std::endl;
                    error = true;
                }
            }
    }

    return !error;
}

// computes the angular part of the jacobian that relates changes in the
// orientation (represented in axis-angle form) of a link to changes in q
void GeneralizedCoordinatesRobotRepresentation::compute_angular_jacobian(const std::shared_ptr<const RB> &rb, Matrix &dRdq) const {
    // the orientation of an RB is obtained by rotating by all the rotation axes
    // up the hierarchy... so the jacobian consists of the world-coords axes
    resize(dRdq, 3, (int)q.size());

    std::vector<int> qIndices;

    int startIndex = getQStartIndexForRB(rb);

    while (startIndex > 2) {
        qIndices.push_back(startIndex);
        startIndex = qParentIndex[startIndex];
    }

    Quaternion Q = Quaternion::Identity();
    // q rotates the rigid body (grand...child) about its world coordinate
    // axis... these will be the entries of dR/dq...
    while (qIndices.size() > 0) {
        int index = qIndices.back();
        Q = Q * getRelOrientationForQ(qIndices.back());
        // this is just the rotation axis in world coordinates, as this is what
        // we are rotating about to get the orientation of the rigid body if
        // everything else is fixed...
        V3D dRdq_i = Q * getQAxis(index);
        dRdq(0, index) = dRdq_i[0];
        dRdq(1, index) = dRdq_i[1];
        dRdq(2, index) = dRdq_i[2];

        qIndices.pop_back();
    }

    /*
            while (rb->pJoint != nullptr) {
                    int jIndex = rb->pJoint->jIndex;
                    //q rotates the rigid body (grand...child) about its world
       coordinate axis... these will be the entries of dp/dq... for (int i = 0;
       i<jointCoordsDimSize[jIndex]; i++) { int index =
       jointCoordStartIndex[jIndex] + jointCoordsDimSize[jIndex] - i - 1; V3D
       dRdq_i = getWorldCoordsAxisForQ(index); dRdq(0, index) = dRdq_i[0];
                            dRdq(1, index) = dRdq_i[1];
                            dRdq(2, index) = dRdq_i[2];
                    }
                    rb = rb->pJoint->parent;
            }

            for (int index = 3; index<6; index++) {
                    V3D dRdq_i = getWorldCoordsAxisForQ(index);
                    dRdq(0, index) = dRdq_i[0];
                    dRdq(1, index) = dRdq_i[1];
                    dRdq(2, index) = dRdq_i[2];
            }
    */
}

// estimates the angular jacobian using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_angular_jacobian(const std::shared_ptr<const RB> &rb, Matrix &dRdq) {
    resize(dRdq, 3, (int)q.size());

    for (int i = 0; i < q.size(); i++) {
        double val = q[i];
        double h = 0.001;

        q[i] = val + h;
        Quaternion R_p = getOrientationFor(rb);

        q[i] = val - h;
        Quaternion R_m = getOrientationFor(rb);

        q[i] = val;
        Quaternion rotate = R_p * R_m.inverse();
        AngleAxisd aa(rotate);
        V3D axis = aa.axis().normalized();
        double angle = aa.angle();
        axis *= angle;

        V3D dRdq_i = axis / (2 * h);
        dRdq(0, i) = dRdq_i[0];
        dRdq(1, i) = dRdq_i[1];
        dRdq(2, i) = dRdq_i[2];
    }
}

bool GeneralizedCoordinatesRobotRepresentation::test_angular_jacobian(const std::shared_ptr<const RB> &rb) {
    Matrix dRdq_analytic, dRdq_estimated;
    compute_angular_jacobian(rb, dRdq_analytic);
    estimate_angular_jacobian(rb, dRdq_estimated);

    //	print("../out/angular_jacobian_analytic.mat", dRdq_analytic);
    //	print("../out/angular_jacobian_estimated.mat", dRdq_estimated);

    bool error = false;

    for (int i = 0; i < dRdq_analytic.rows(); i++)
        for (int j = 0; j < dRdq_analytic.cols(); j++) {
            double err = dRdq_analytic(i, j) - dRdq_estimated(i, j);
            if (fabs(err) > 0.0001) {
                std::cout << "Error at: " << i << " " << j << ": analytic: " << dRdq_analytic(i, j) << " estimated " << dRdq_estimated(i, j)
                          << " error: " << err << std::endl;
                error = true;
            }
        }

    return !error;
}

// computes the d(Jw)/dqi
bool GeneralizedCoordinatesRobotRepresentation::compute_dangular_jacobian_dqi(const std::shared_ptr<const RB> &rb, Matrix &ddRdqdqi, int q_i) const {
    resize(ddRdqdqi, 3, (int)q.size());
    if (q_i < 3)
        return false;

    int startIndex = getQStartIndexForRB(rb);

    std::vector<int> qIndices;

    // if q_i is not one of the ancestors of rb, then it means the jacobian dpdq
    // does not depent on it, so check first...
    int qIndex = startIndex;
    bool isAncestor = false;

    Quaternion Q_qi = Quaternion::Identity();

    while (qIndex > 2) {
        qIndices.push_back(qIndex);

        if (q_i == qIndex) {
            isAncestor = true;
        }
        if (isAncestor)
            Q_qi = getRelOrientationForQ(qIndex) * Q_qi;
        qIndex = qParentIndex[qIndex];
    }

    if (!isAncestor)
        return false;

    V3D q_i_rotAxisWorld = Q_qi * getQAxis(q_i);

    Quaternion Q = Quaternion::Identity();
    // q rotates the rigid body (grand...child) about its world coordinate
    // axis... these will be the entries of dR/dq...
    while (qIndices.size() > 0) {
        int index = qIndices.back();
        Q = Q * getRelOrientationForQ(qIndices.back());

        if (index > q_i) {
            V3D derivativeVal = q_i_rotAxisWorld.cross(Q * getQAxis(index));

            ddRdqdqi(0, index) = derivativeVal(0);
            ddRdqdqi(1, index) = derivativeVal(1);
            ddRdqdqi(2, index) = derivativeVal(2);
        }
        qIndices.pop_back();
    }

    /*

            //input is valid, so we must compute this derivative...
            int loopIndex = startIndex;
            //2 here is the index of the first translational DOF of the root
            while (loopIndex > q_i) {
    //		Logger::consolePrint("NEW: adding contribution for index: %d\n",
    loopIndex); V3D derivativeVal =
    getWorldCoordsAxisForQ(q_i).cross(getWorldCoordsAxisForQ(loopIndex));
                    ddRdqdqi(0, loopIndex) = derivativeVal(0);
                    ddRdqdqi(1, loopIndex) = derivativeVal(1);
                    ddRdqdqi(2, loopIndex) = derivativeVal(2);

                    loopIndex = qParentIndex[loopIndex];
            }
    */
    return true;
}

// estimates the change of angular jacobian with respect to q_i using finite
// differences
void GeneralizedCoordinatesRobotRepresentation::estimate_dangular_jacobian_dqi(const std::shared_ptr<const RB> &rb, Matrix &ddRdq_dqi, int q_i) {
    resize(ddRdq_dqi, 3, (int)q.size());
    Matrix dRdq_p, dRdq_m;
    dRdq_p = ddRdq_dqi;
    dRdq_m = ddRdq_dqi;

    double val = q[q_i];
    double h = 0.0001;

    q[q_i] = val + h;
    compute_angular_jacobian(rb, dRdq_p);

    q[q_i] = val - h;
    compute_angular_jacobian(rb, dRdq_m);

    q[q_i] = val;

    ddRdq_dqi = (dRdq_p - dRdq_m) / (2 * h);
}

bool GeneralizedCoordinatesRobotRepresentation::test_angular_jacobian_derivatives(const std::shared_ptr<const RB> &rb) {
    Matrix ddRdqdq_analytic, ddRdqdq_estimated;

    bool error = false;

    for (int k = 0; k < getDimensionSize(); k++) {
        compute_dangular_jacobian_dqi(rb, ddRdqdq_analytic, k);
        estimate_dangular_jacobian_dqi(rb, ddRdqdq_estimated, k);

        //		print("../out/angular_jacobian_dqi_analytic.mat",
        // ddRdqdq_analytic);
        // print("../out/angular_jacobian_dqi_estimated.mat",
        // ddRdqdq_estimated);

        for (int i = 0; i < ddRdqdq_analytic.rows(); i++)
            for (int j = 0; j < ddRdqdq_analytic.cols(); j++) {
                double err = ddRdqdq_analytic(i, j) - ddRdqdq_estimated(i, j);
                if (fabs(err) > 0.0001) {
                    std::cout << "Error when computing ddRdqdq " << k << " at: " << i << " " << j << ": analytic: " << ddRdqdq_analytic(i, j) << " estimated "
                              << ddRdqdq_estimated(i, j) << " error: " << err << std::endl;
                    error = true;
                    //					exit(0);
                }
            }
    }

    return !error;
}

// computes the generalized mass matrix for rigidbody rb: M = J'McJ, where Mc is
// a 6x6, local coordinates matrix, and M is a |q|x|q| generalized matrix...
void GeneralizedCoordinatesRobotRepresentation::computeMassMatrixForRB(const std::shared_ptr<const RB> &rb, Matrix &massMatrix) const {
    /*
    M = J'McJ
    Mc = [	m 0 0  0
            0 m 0  0
            0 0 m  0
            0 0 0 MoI]
    */
    resize(massMatrix, q.size(), q.size());

    Matrix dRdq, dpdq;

    // the MOI here needs to be in global coordinates
    Matrix3x3 MoI = rb->rbProps.getMOI(getOrientationFor(rb).toRotationMatrix());

    compute_dpdq(P3D(0, 0, 0), rb, dpdq);
    compute_angular_jacobian(rb, dRdq);

    massMatrix.noalias() = rb->rbProps.mass * dpdq.transpose() * dpdq;
    massMatrix.noalias() += dRdq.transpose() * MoI * dRdq;
}

// computes the mass matrix for the whole robot
void GeneralizedCoordinatesRobotRepresentation::computeMassMatrix(Matrix &massMatrix) const {
    resize(massMatrix, q.size(), q.size());

    computeMassMatrixForRB(robot->root, massMatrix);

    Matrix curMassMatrix;
    for (uint i = 0; i < robot->jointList.size(); ++i) {
        computeMassMatrixForRB(robot->jointList[i]->child, curMassMatrix);
        massMatrix += curMassMatrix;
    }
}

// computes dpdq_dot, dpdq_dot = sigma(dpdq_dqi * qiDot): JDot = dJ/dq * qDot
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq_dot(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq_dot) const {
    Matrix ddpdq_dqi;

    resize(dpdq_dot, 3, (int)q.size());
    dpdq_dot.setZero();
    for (int i = 0; i < (int)q.size(); ++i) {
        compute_ddpdq_dqi(p, rb, ddpdq_dqi, i);
        dpdq_dot += ddpdq_dqi * qDot[i];
    }
}

// computed dRdq_dot, dRdq_dot = sigma(dRdq_dqi * qiDot): JDot = dJ/dq * qDot
void GeneralizedCoordinatesRobotRepresentation::compute_angular_jacobian_dot(const std::shared_ptr<const RB> &rb, Matrix &dRdq_dot) const {
    Matrix ddRdq_dqi;

    resize(dRdq_dot, 3, (int)q.size());
    dRdq_dot.setZero();

    for (int i = 0; i < (int)q.size(); ++i) {
        compute_dangular_jacobian_dqi(rb, ddRdq_dqi, i);
        dRdq_dot += ddRdq_dqi * qDot[i];
    }
}

// computes the Coriolis Vector for the whole robot
void GeneralizedCoordinatesRobotRepresentation::computeCoriolisAndCentrifugalForcesTerm(dVector &C) const {
    Matrix coriolisMatrix;
    resize(coriolisMatrix, (int)q.size(), (int)q.size());

    Matrix curCoriolisMatrix;
    computeCoriolisMatrix(robot->root, coriolisMatrix);
    for (uint i = 0; i < robot->jointList.size(); ++i) {
        computeCoriolisMatrix(robot->jointList[i]->child, curCoriolisMatrix);
        coriolisMatrix += curCoriolisMatrix;
    }
    C = coriolisMatrix * qDot;
}

// computes the rigidbody rb's contribution (corioli and centrifugal force part)
// to the coriolisMatrix_ term, CMatrix(q, qDot) = (J'McJDot + J'[w]McJ)
void GeneralizedCoordinatesRobotRepresentation::computeCoriolisMatrix(const std::shared_ptr<const RB> &rb, Matrix &coriolisMatrix) const {
    /*
        [w] = [0	0
               0 [Jw*qDot] ]
    */
    resize(coriolisMatrix, (int)q.size(), (int)q.size());
    coriolisMatrix.setZero();

    // calculate linear part contribution
    Matrix dpdq, dpdq_dot;
    compute_dpdq(P3D(0, 0, 0), rb, dpdq);
    compute_dpdq_dot(P3D(0, 0, 0), rb, dpdq_dot);

    coriolisMatrix.noalias() = rb->rbProps.mass * dpdq.transpose() * dpdq_dot;
    //   print("dpdq_dot", dpdq_dot);
    // calculate angular part contribution
    Matrix dRdq, dRdq_dot, omega;
    // the MOI here needs to be in global coordinates
    Matrix3x3 MoI = rb->rbProps.getMOI(getOrientationFor(rb).toRotationMatrix());
    compute_angular_jacobian(rb, dRdq);
    compute_angular_jacobian_dot(rb, dRdq_dot);
    omega = getSkewSymmetricMatrix(V3D(dRdq * qDot));
    coriolisMatrix.noalias() += (dRdq.transpose() * MoI * dRdq_dot + dRdq.transpose() * omega * MoI * dRdq);
}

void testGeneralizedCoordinateRepresentation(const std::shared_ptr<Robot> &robot) {
    std::cout << "testing generalized coordinates representation..." << std::endl;

    // make sure we project errors introduced by physics engine (i.e. hinge
    // joints not rotating only about their axis)
    robot->fixJointConstraints();

    // test out projections between robot state and generalized coordinates...
    RobotState robotState1(*robot);

    GeneralizedCoordinatesRobotRepresentation gcrrNew(robot);

    dVector q1, q1Dot;
    gcrrNew.getQ(q1);
    gcrrNew.getQDot(q1Dot);

    gcrrNew.syncRobotStateWithGeneralizedCoordinates();
    RobotState robotState2(*robot);

    RobotState robotState3(*robot);
    gcrrNew.getReducedRobotState(robotState3);
    robot->setState(robotState3);
    gcrrNew.syncGeneralizedCoordinatesWithRobotState();

    dVector q2, q2Dot;
    gcrrNew.getQ(q2);
    gcrrNew.getQDot(q2Dot);

    if (!(robotState1 == robotState2)) {
        std::cout << "TESTING GENERALIZED COORDINATES: robot state is not the "
                     "same after projection..."
                  << std::endl;
    }

    if ((q1 - q2).norm() > 1e-8) {
        std::cout << "TESTING GENERALIZED COORDINATES: generalized positions "
                     "(q) are not the same before and after projection. Error: "
                  << (q1 - q2).norm() << std::endl;
        //		print("qErr.txt", q1 - q2);
        //		exit(0);
    }

    //	print("qErr.txt", q1 - q2);
    //	exit(0);

    if ((q1Dot - q2Dot).norm() > 1e-8) {
        std::cout << "TESTING GENERALIZED COORDINATES: generalized velocities (qDot) "
                     "are not the same before and after projection. Error: "
                  << (q1Dot - q2Dot).norm() << std::endl;
    }

    // test forward kinematics (world pos of points on RBs, velocity of points,
    // etc)...
    for (int i = 0; i < robot->getJointCount(); i++) {
        P3D point = P3D(0, 0, 0) + getRandomUnitVector() * 0.2;
        P3D wc1 = robot->getJoint(i)->child->getWorldCoordinates(point);
        P3D wc2 = gcrrNew.getWorldCoordinates(point, robot->getJoint(i)->child);
        if (V3D(wc1, wc2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: world coordinates "
                         "of point on rigid body do not match up... error: "
                      << V3D(wc1, wc2).norm() << std::endl;

        wc1 = robot->getJoint(i)->parent->getWorldCoordinates(point);
        wc2 = gcrrNew.getWorldCoordinates(point, robot->getJoint(i)->parent);
        if (V3D(wc1, wc2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: world coordinates "
                         "of point on rigid body do not match up... error: "
                      << V3D(wc1, wc2).norm() << std::endl;

        V3D vec = getRandomUnitVector() * 0.2;
        V3D vc1 = robot->getJoint(i)->child->getWorldCoordinates(vec);
        V3D vc2 = gcrrNew.getWorldCoordinates(vec, robot->getJoint(i)->child);

        if ((vc1 - vc2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: world coordinates "
                         "of vectors on rigid body do not match up... error: "
                      << (vc1 - vc2).norm() << std::endl;

        vc1 = robot->getJoint(i)->parent->getWorldCoordinates(vec);
        vc2 = gcrrNew.getWorldCoordinates(vec, robot->getJoint(i)->parent);

        if ((vc1 - vc2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: world coordinates "
                         "of vectors on rigid body do not match up... error: "
                      << (vc1 - vc2).norm() << std::endl;

        V3D wv1 = robot->getJoint(i)->child->getVelocityForPoint_local(point);
        V3D wv2 = gcrrNew.getVelocityFor(point, robot->getJoint(i)->child);
        if ((wv1 - wv2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: velocities of point "
                         "on rigid body do not match up... error: "
                      << (wv1 - wv2).norm() << std::endl;

        wv1 = robot->getJoint(i)->child->getVelocityForPoint_local(P3D());
        wv2 = gcrrNew.getVelocityFor(P3D(0, 0, 0), robot->getJoint(i)->child);
        if ((wv1 - wv2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: velocities of rigid "
                         "body do not match up... error: "
                      << (wv1 - wv2).norm() << std::endl;

        wv1 = robot->getJoint(i)->child->getAngularVelocity();
        wv2 = gcrrNew.getAngularVelocityFor(robot->getJoint(i)->child);
        if ((wv1 - wv2).norm() > 1e-8)
            std::cout << "TESTING GENERALIZED COORDINATES: angular velocities "
                         "of rigid body do not match up... error: "
                      << (wv1 - wv2).norm() << std::endl;

        Quaternion r1 = robot->getJoint(i)->child->getOrientation();
        Quaternion r2 = gcrrNew.getOrientationFor(robot->getJoint(i)->child);
        if (!sameRotation(r1, r2)) {
            std::cout << "TESTING GENERALIZED COORDINATES: orientations of "
                         "rigid body do not match up... error: "
                      << r1.w() << " / " << r2.w() << std::endl;
        }

        if (!gcrrNew.test_linear_jacobian(point, robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "(P3D) does not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian(point, robot->getJoint(i)->parent))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "(P3D) does not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian(vec, robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "(V3D) does not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian(vec, robot->getJoint(i)->parent))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "(V3D) does not match FD..."
                      << std::endl;
        ;

        if (!gcrrNew.test_angular_jacobian(robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: angular jacobian "
                         "does not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian_derivatives(point, robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "derivatives do not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian_derivatives(vec, robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "derivatives do not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian_derivatives(point, robot->getJoint(i)->parent))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "derivatives do not match FD..."
                      << std::endl;

        if (!gcrrNew.test_linear_jacobian_derivatives(vec, robot->getJoint(i)->parent))
            std::cout << "TESTING GENERALIZED COORDINATES: linear jacobian "
                         "derivatives do not match FD..."
                      << std::endl;

        if (!gcrrNew.test_angular_jacobian_derivatives(robot->getJoint(i)->child))
            std::cout << "TESTING GENERALIZED COORDINATES: angular jacobian "
                         "derivatives do not match FD..."
                      << std::endl;
    }

    robot->setState(robotState1);
}

}  // namespace crl::loco
