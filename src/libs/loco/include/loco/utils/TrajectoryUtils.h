//
// Created by Dongho Kang on 19.12.21.
//

#ifndef CRL_LOCO_TRAJECTORY_H
#define CRL_LOCO_TRAJECTORY_H

#include "crl-basic/utils/trajectory.h"

namespace crl::loco {

template <typename T>
void addTimeOffset(GenericTrajectory<T>& traj, double t) {
    for (uint i = 0; i < traj.tValues.size(); i++) {
        traj.tValues[i] += t;
    }
}

template <typename T>
void tidyUpBefore(GenericTrajectory<T>& traj, double t) {
    while (traj.getKnotCount() > 0 && traj.getKnotPosition(0) < t)
        traj.removeKnot(0);
}

template <typename T>
void tidyUpAfter(GenericTrajectory<T>& traj, double t) {
    while (traj.getKnotCount() > 0 && traj.getKnotPosition(traj.getKnotCount() - 1) >= t)
        traj.removeKnot(traj.getKnotCount() - 1);
}

template <typename T>
T cubic_bezier_interpolation(T yStart, T yEnd, double x) {
    T yDiff = yEnd - yStart;
    double bezier = x * x * x + 3 * (x * x * (1 - x));
    return yStart + bezier * yDiff;
}

template <typename T>
T cubic_bezier_first_derivative(T yStart, T yEnd, double x) {
    T yDiff = yEnd - yStart;
    double bezier = 6 * x * (1 - x);
    return bezier * yDiff;
}

template <typename T>
T cubic_bezier_second_derivative(T yStart, T yEnd, double x) {
    T yDiff = yEnd - yStart;
    double bezier = 6 - 12 * x;
    return bezier * yDiff;
}

inline P3D evaluateBezierPosition(const P3D& lastContactPos, const P3D& nextContactPos, double swingHeight, double percentageOfTimeElapsed) {
    P3D p = P3D() + cubic_bezier_interpolation(V3D(lastContactPos), V3D(nextContactPos), percentageOfTimeElapsed);
    if (percentageOfTimeElapsed < 0.5) {
        // take off
        double h = cubic_bezier_interpolation(lastContactPos.y, swingHeight, percentageOfTimeElapsed * 2);
        p.y = h;
        return p;
    } else {
        // landing
        double h = cubic_bezier_interpolation(swingHeight, nextContactPos.y, percentageOfTimeElapsed * 2 - 1);
        p.y = h;
        return p;
    }
}

inline V3D evaluateBezierVelocity(const P3D& lastContactPos, const P3D& nextContactPos, double swingHeight, double percentageOfTimeElapsed,
                                  double swingDuration) {
    V3D v = cubic_bezier_first_derivative(V3D(lastContactPos), V3D(nextContactPos), percentageOfTimeElapsed);
    v = v / swingDuration;

    if (percentageOfTimeElapsed < 0.5) {
        // take off
        double vy = cubic_bezier_first_derivative(lastContactPos.y, swingHeight, percentageOfTimeElapsed * 2);
        vy = vy * 2 / swingDuration;
        v.y() = vy;
        return v;
    } else {
        // landing
        double vy = cubic_bezier_first_derivative(swingHeight, nextContactPos.y, percentageOfTimeElapsed * 2 - 1);
        vy = vy * 2 / swingDuration;
        v.y() = vy;
        return v;
    }
}

inline V3D evaluateBezierAcceleration(const P3D& lastContactPos, const P3D& nextContactPos, double swingHeight, double percentageOfTimeElapsed,
                                      double swingDuration) {
    V3D a = cubic_bezier_second_derivative(V3D(lastContactPos), V3D(nextContactPos), percentageOfTimeElapsed);
    a = a / swingDuration / swingDuration;

    if (percentageOfTimeElapsed < 0.5) {
        // take off
        double ay = cubic_bezier_second_derivative(lastContactPos.y, swingHeight, percentageOfTimeElapsed * 2);
        ay = ay * 4 / swingDuration / swingDuration;
        a.y() = ay;
        return a;
    } else {
        // landing
        double ay = cubic_bezier_second_derivative(swingHeight, nextContactPos.y, percentageOfTimeElapsed * 2 - 1);
        ay = ay * 4 / swingDuration / swingDuration;
        a.y() = ay;
        return a;
    }
}

}  // namespace crl::loco

#endif  //CRL_LOCO_TRAJECTORY_H
