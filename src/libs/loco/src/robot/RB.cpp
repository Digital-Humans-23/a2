#include "loco/robot/RB.h"

namespace crl::loco {

Matrix3x3 RB::getWorldMOI() const {
    return rbProps.getMOI(state.orientation);
}

Matrix3x3 RB::getWorldMOI(const V3D &v) const {
    Matrix3x3 MOIw = getWorldMOI();
    MOIw(0, 0) += rbProps.mass * (v.y() * v.y() + v.z() * v.z());
    MOIw(0, 1) -= rbProps.mass * v.x() * v.y();
    MOIw(0, 2) -= rbProps.mass * v.x() * v.z();
    MOIw(1, 0) -= rbProps.mass * v.x() * v.y();
    MOIw(1, 1) += rbProps.mass * (v.x() * v.x() + v.z() * v.z());
    MOIw(1, 2) -= rbProps.mass * v.y() * v.z();
    MOIw(2, 0) -= rbProps.mass * v.x() * v.z();
    MOIw(2, 1) -= rbProps.mass * v.y() * v.z();
    MOIw(2, 2) += rbProps.mass * (v.x() * v.x() + v.y() * v.y());
    return MOIw;
}

#define UPDATE_RAY_INTERSECTION(P1, P2)                                            \
    if (ray.getDistanceToSegment((P1), (P2), &tmpIntersectionPoint) < cylRadius) { \
        double t = ray.getRayParameterFor(tmpIntersectionPoint);                   \
        if (t < tMin) {                                                            \
            intersectionPoint = ray.origin + ray.dir * t;                          \
            tMin = t;                                                              \
        }                                                                          \
    }

bool RB::getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint, bool checkMeshes, bool checkSkeleton) {
    P3D tmpIntersectionPoint;
    double tMin = DBL_MAX;
    double t = tMin;
    // we will check all meshes and all cylinders that are used to show the
    // abstract view...

    // meshes first...

    if (checkMeshes)
        for (uint i = 0; i < rbProps.models.size(); i++) {
            RigidTransformation meshTransform(state.orientation, state.pos);
            meshTransform *= rbProps.models[i].localT;

            rbProps.models[i].position = meshTransform.T;
            rbProps.models[i].orientation = meshTransform.R;

            if (rbProps.models[i].hitByRay(ray.origin, ray.dir, t)) {
                if (t < tMin) {
                    intersectionPoint = ray.origin + ray.dir * t;
                    tMin = t;
                }
            }
        }

    if (checkSkeleton) {
        // and now the cylinders...
        double cylRadius = rbProps.abstractViewCylRadius;

        if (pJoint != nullptr)
            UPDATE_RAY_INTERSECTION(pJoint->getWorldPosition(), state.pos);

        for (uint i = 0; i < cJoints.size(); i++)
            UPDATE_RAY_INTERSECTION(state.pos, cJoints[i]->getWorldPosition());

        if (cJoints.size() == 0) {
            for (uint i = 0; i < rbProps.endEffectorPoints.size(); i++) {
                P3D startPos = getWorldCoordinates(P3D(0, 0, 0));
                P3D endPos = getWorldCoordinates(rbProps.endEffectorPoints[i].endEffectorOffset);
                UPDATE_RAY_INTERSECTION(startPos, endPos);
            }
        }
    }

    return tMin < DBL_MAX / 2.0;
}

}  // namespace crl::loco
