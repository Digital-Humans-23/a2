//
// Created by Dongho Kang on 21.12.21.
//

#ifndef CRL_LOCO_MATHUTILS_H
#define CRL_LOCO_MATHUTILS_H

namespace crl::loco {

template <typename T>
T getTargetAcceleration_explicitPD(const T &posError, const T &velError, double kp, double kd) {
    return -posError * kp - velError * kd;
}

template <typename T>
T getTargetAcceleration_implicitPD(const T &curPosError, const T &curVelError, const T &curVel, double kp, double kd, double dt) {
    // what we know:
    //  P_t+1 = P_t + h * V_t+1
    //	V_t+1 = V_t + h * a
    //	a = -kp * P_t+1 - kd * V_t+1
    // plug the expressions for P_t+1 and V_t+1 into a and solve...
    return -(curPosError * kp + curVelError * kd + curVel * dt * kp) / (1 + dt * dt * kp + dt * kd);
}

template <typename T>
T getTargetAcceleration_implicitPD(const T &curPos, const T &targetPos, const T &curVel, const T &targetVel, double kp, double kd, double dt) {
    return getTargetAcceleration_implicitPD(curPos - targetPos, curVel - targetVel, curVel, kp, kd, dt);
}

template <typename T>
T getTargetAcceleration_posConstraint(const T &targetPos, const T &curPos, const T &curVel, double dt) {
    // what we know:
    //	P_t+1 = P_t + h * V_t+1
    //	V_t+1 = V_t + h * a
    // if we want P_t+1 to be targetPos (e.g. position error vanishes)
    // then a = (targetPos - P_t - h * vt) / (h * h)
    return (targetPos - curPos - curVel * dt) / (dt * dt);
}

inline Quaternion getOrientationFromRollPitchYawAngles(const V3D &forward, double roll, double pitch, double yaw) {
    return getRotationQuaternion(yaw, RBGlobals::worldUp) * getRotationQuaternion(pitch, RBGlobals::worldUp.cross(forward)) *
           getRotationQuaternion(roll, forward);
}

inline void computeRollPitchYawAnglesFromQuaternion(const Quaternion &q, const V3D &forward, double &roll, double &pitch, double &yaw) {
    computeEulerAnglesFromQuaternion(q, forward, RBGlobals::worldUp.cross(forward), RBGlobals::worldUp, roll, pitch, yaw);
}

inline double computeHeadingFromQuaternion(const Quaternion &q) {
    double roll, pitch, yaw;
    V3D v1, v2;
    getOrthogonalVectors(RBGlobals::worldUp, v1, v2);
    computeEulerAnglesFromQuaternion(q, v1, v2, RBGlobals::worldUp, roll, pitch, yaw);
    return yaw;
}

inline Matrix pseudoInverse(const Matrix &matrix, double sigmaThreshold = 1e-3) {
    // this bit of code is originally from https://github.com/mit-biomimetics/Cheetah-Software/blob/c71c5a138d3e418cc833e94e25357ceea8955daa/common/include/Utilities/pseudoInverse.h
    Matrix inv;
    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
        inv.resize(1, 1);
        if (matrix.coeff(0, 0) > sigmaThreshold) {
            inv.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
        } else {
            inv.coeffRef(0, 0) = 0.0;
        }
        return inv;
    }

    Eigen::JacobiSVD<Matrix> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // not sure if we need to svd.sort()... probably not
    const uint nrows(svd.singularValues().rows());
    Matrix invS = Matrix ::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
        if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        } else {
            // well this shouldn't be happened...
            invS.coeffRef(ii, ii) = 1.0 / sigmaThreshold;
        }
    }
    inv = svd.matrixV() * invS * svd.matrixU().transpose();
    return inv;
}

inline Matrix weightedPseudoInverse(const Matrix &matrix, const Matrix &wInv, double threshold = 1e-4) {
    Matrix lambda(matrix * wInv * matrix.transpose());
    Matrix lambda_inv = pseudoInverse(lambda, threshold);
    return wInv * matrix.transpose() * lambda_inv;
}

}  // namespace crl::loco

#endif  //CRL_LOCO_MATHUTILS_H
