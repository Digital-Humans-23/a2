#pragma once

#include <crl-basic/utils/mathUtils.h>

#include "loco/robot/Robot.h"
#include "loco/robot/RobotState.h"

namespace crl::loco {

/**
 * This class implements a reduced representation of an robot (i.e. we represent
 * the configuration using vectors q and qDot, which represent the generalized
 * coordinates of the robot.
 */

class GeneralizedCoordinatesRobotRepresentation {
private:
    // this is the reference to the robot whose reduced representation is being
    // stored
    std::shared_ptr<Robot> robot = nullptr;

    //--- PARAMETERS THAT DEFINE THE MORPHOLOGY OF THE ROBOT
    // for each joint, the index at which its generalized coordinates start can
    // be read off from this array
    std::vector<int> jointCoordStartIndex;
    // and for each joint, this is how many generalized coords we need to
    // represent it (depending on what type of joints these are)...
    std::vector<int> jointCoordsDimSize;
    // this is all one big, tree-based hierarchichal transformation. Keep track
    // of the hierarchy, by storing, for each degree of freedom, the index of
    // the parent dof
    std::vector<int> qParentIndex;
    // for every q, keep track of the joint that it corresponds to...
    std::vector<int> jointIndexForQ;

    //--- PARAMETERS THAT DEFINE THE STATE OF THE ROBOT - NEED TO BE UPDATED
    // EVERY TIME THE STATE OF THE ROBOT CHANGES generalized coordinates - pos
    // and velocities
    dVector q, qDot;

public:
    /** the constructor */
    GeneralizedCoordinatesRobotRepresentation(const std::shared_ptr<Robot> &a);

    /** the destructor */
    virtual ~GeneralizedCoordinatesRobotRepresentation(void) {}

    /**
     * sets up the whole structure of the robot.
     */
    void setupGeneralizedCoordinatesStructure();

    /**
     * returns the local coord vector from the parent of q(qIndex) to q(qIndex).
     */
    V3D getOffsetFromParentToQ(int qIndex) const;

    /**
     * returns the axis correponding to the indexed generalized coordinate,
     * expressed in local coordinates.
     */
    V3D getQAxis(int qIndex) const;

    /**
     * returns the relative orientation that a specific q dof induces...
     */
    Quaternion getRelOrientationForQ(int qIndex) const;

    /**
     * returns the global orientation associated with a specific dof q...
     */
    Quaternion getWorldRotationForQ(int qIndex) const;

    /**
     * returns the translation or rotation axis for a specific dof q...
     */
    V3D getWorldCoordsAxisForQ(int qIndex) const;

    /**
     * returns the index of the last (e.g. highest) dof that controls the
     * position/orientation of the rb.
     */
    int getQStartIndexForRB(const std::shared_ptr<const RB> &rb) const;

    /**
     * returns the local position of the point that rb pivots about (i.e.
     * location of the parent joint), in coordinate frame of rb
     */
    P3D getPivotPointLocalPosition(const std::shared_ptr<const RB> &rb) const;

    /* projection to/from world coords and generalized coords */
    void projectWorldCoordsValuesIntoGeneralizedSpace(const V3D &linearVal, const V3D &angularVal, const std::vector<V3D> &jointVal, dVector &generalizedArray);

    void projectVectorOnGeneralizedCoordsAxes(const V3D &vector, const V3D &a, const V3D &b, const V3D &c, double &aVal, double &bVal, double &cVal);
    void projectVectorOnGeneralizedCoordsAxes(const V3D &vector, const V3D &a, const V3D &b, double &aVal, double &bVal);
    void projectVectorOnGeneralizedCoordsAxes(const V3D &vector, const V3D &a, double &aVal);

    /**
     * computes the rigidbody rb's contribution (corioli and centrifugal force
     * part) to the coriolisMatrix_ term, CMatrix(q, qDot) = (J'McJDot +
     * J'[w]McJ)
     */
    void computeCoriolisMatrix(const std::shared_ptr<const RB> &rb, Matrix &coriolisMatrix) const;

    /**
     * computes the generalized mass matrix for rigidbody rb: M = J'McJ, where
     * Mc is a 6x6, local coordinates matrix, and M is a |q|x|q| matrix...
     */
    void computeMassMatrixForRB(const std::shared_ptr<const RB> &rb, Matrix &massMatrix) const;

    /**
     * given the current state of the generalized representation, output the
     * reduced state of the robot
     */
    void getReducedRobotState(RobotState &state) const;

    /**
     * updates robot state given current q and qDot values...
     */
    void syncRobotStateWithGeneralizedCoordinates() const;

    /**
     * updates q and qDot given current state of robot...
     */
    void syncGeneralizedCoordinatesWithRobotState();

    /**
     * integrates state forward in time using input accelerations...
     */
    void integrateGenerlizedAccelerationsForwardInTime(const dVector &a, double dt);

    /**
     * sets the current q values
     */
    void setQ(const dVector &qNew);

    /**
     * gets the current q values
     */
    void getQ(dVector &q_copy) const;

    /**
     * gets the q value from robot state
     */
    void getQFromReducedState(const RobotState &rs, dVector &q_copy);

    /**
     * gets the q and qdot_ value from robot state
     */
    void getQAndQDotFromReducedState(const RobotState &rs, dVector &q_copy, dVector &qDot_copy);

    /**
     * sets the current qDot values
     */
    void setQDot(const dVector &qDot);

    /**
     * gets the current qDot values
     */
    void getQDot(dVector &qDot_copy) const;

    /**
     * returns the world coordinates for point p, which is specified in the
     * local coordinates of rb (relative to its COM). I.e. p(q)
     */
    P3D getWorldCoordinates(const P3D &p, const std::shared_ptr<const RB> &rb) const;

    /**
     *  returns the world coordinates for vector b, which is specified in the
     * local coordinates of rb
     */
    V3D getWorldCoordinates(const V3D &v, const std::shared_ptr<const RB> &rb) const;

    /**
     * returns the velocity (world coordinates) of the point p, which is
     * specified in the local coordinates of rb (relative to its COM). I.e. p(q)
     */
    V3D getVelocityFor(const P3D &p, const std::shared_ptr<const RB> &rb) const;

    /**
     * returns the angular velocity (world coordinates) for the rigid body rb
     */
    V3D getAngularVelocityFor(const std::shared_ptr<const RB> &rb) const;

    /**
     * returns the world-relative orientation for rb
     */
    Quaternion getOrientationFor(const std::shared_ptr<const RB> &rb) const;

    /**
     * computes the jacobian dp/dq that tells you how the world coordinates of p
     * change with q. p is expressed in the local coordinates of rb
     */
    void compute_dpdq(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq) const;

    void compute_dvdq(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &dvdq) const;

    /**
     * computes dpdq_dot, dpdq_dot = sigma(dpdq_dqi * qiDot) : JDot = dJ/dq *
     * qDot
     */
    void compute_dpdq_dot(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq_dot) const;

    /**
     * computed dRdq_dot, dRdq_dot = sigma(dRdq_dqi * qiDot) : JDot = dJ/dq *
     * qDot
     */
    void compute_angular_jacobian_dot(const std::shared_ptr<const RB> &rb, Matrix &dRdq_dot) const;

    /**
     * estimates the jacobian dp/dq using finite differences
     */
    void estimate_linear_jacobian(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &dpdq);

    /**
     * estimates the jacobian dv/dq using finite differences
     */
    void estimate_linear_jacobian(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &dvdq);

    bool test_linear_jacobian(const P3D &p, const std::shared_ptr<const RB> &rb);
    bool test_linear_jacobian(const V3D &v, const std::shared_ptr<const RB> &rb);

    /**
     * computes the angular part of the jacobian, that, roughly speaking,
     * relates changes in the orientation of a link to changes in q
     */
    void compute_angular_jacobian(const std::shared_ptr<const RB> &rb, Matrix &dRdq) const;

    /**
     * estimates the angular jacobian using finite differences
     */
    void estimate_angular_jacobian(const std::shared_ptr<const RB> &rb, Matrix &dRdq);

    bool test_angular_jacobian(const std::shared_ptr<const RB> &rb);

    /**
     * computes the matrix that tells you how the jacobian dp/dq changes with
     * respect to q_i. Returns true if it contains non-zero elements, false
     * otherwise
     */
    bool compute_ddpdq_dqi(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &ddpdq_dqi, int q_i) const;

    /**
     * computes the matrix that tells you how the jacobian dv/dq changes with
     * respect to q_i. Returns true if it contains non-zero elements, false
     * otherwise
     */
    bool compute_ddvdq_dqi(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &ddvdq_dqi, int q_i) const;

    /**
     * estimates the change of dp/dq with respect to q_i
     */
    void estimate_ddpdq_dqi(const P3D &p, const std::shared_ptr<const RB> &rb, Matrix &ddpdq_dqi, int q_i);

    /**
     * estimates the change of dv/dq with respect to q_i
     */
    void estimate_ddvdq_dqi(const V3D &v, const std::shared_ptr<const RB> &rb, Matrix &ddvdq_dqi, int q_i);

    bool test_linear_jacobian_derivatives(const P3D &p, const std::shared_ptr<const RB> &rb);
    bool test_linear_jacobian_derivatives(const V3D &p, const std::shared_ptr<const RB> &rb);

    /**
     * computes the d(Jw)/dqi. Returns true if it contains non-zero elements,
     * false otherwise
     */
    bool compute_dangular_jacobian_dqi(const std::shared_ptr<const RB> &rb, Matrix &ddRdqdqi, int q_i) const;

    /**
     * estimates the change of angular jacobian with respect to q_i using finite
     * differences
     */
    void estimate_dangular_jacobian_dqi(const std::shared_ptr<const RB> &rb, Matrix &ddRdq_dqi, int q_i);

    bool test_angular_jacobian_derivatives(const std::shared_ptr<const RB> &rb);

    /**
     * computes the mass matrix for the whole robot
     */
    void computeMassMatrix(Matrix &massMatrix) const;

    /**
     * computes the Coriolis Vector for the whole robot
     */
    void computeCoriolisAndCentrifugalForcesTerm(dVector &C) const;

    /**
     * returns the qIndex at which this joint starts
     */
    int getQIndexForJoint(const std::shared_ptr<const RBJoint> &joint) const;

    inline int getQIndexForJoint(int jIndex) const {
        return jointCoordStartIndex[jIndex];
    }

    inline int getJointIndexForQ(int QIndex) const {
        return jointIndexForQ[QIndex];
    }

    inline std::shared_ptr<RBJoint> getJointForQ(int QIndex) {
        return robot->jointList[getJointIndexForQ(QIndex)];
    }

    inline const std::shared_ptr<RBJoint> &getJointForQ(int QIndex) const {
        return robot->jointList[getJointIndexForQ(QIndex)];
    }

    inline int getDimensionSize() const {
        return (int)q.size();
    }

    double getQVal(int idx) const {
        return q[idx];
    }

    double getQDotVal(int idx) const {
        return qDot[idx];
    }
};

void testGeneralizedCoordinateRepresentation(const std::shared_ptr<Robot> &robot);

// use alias GCRR.
typedef GeneralizedCoordinatesRobotRepresentation GCRR;

}  // namespace crl::loco