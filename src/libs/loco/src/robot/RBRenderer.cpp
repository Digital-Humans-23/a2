#include "loco/robot/RBRenderer.h"

#include "crl-basic/gui/renderer.h"

namespace crl::loco {

void RBRenderer::drawCoordFrame(const std::shared_ptr<const RB> &rb, const gui::Shader &shader) {
    drawArrow3d(rb->getWorldCoordinates(P3D()), V3D(rb->getWorldCoordinates(V3D(1, 0, 0)) * 0.1), 0.01, shader, V3D(1.0, 0.0, 0.0));
    drawArrow3d(rb->getWorldCoordinates(P3D()), V3D(rb->getWorldCoordinates(V3D(0, 1, 0)) * 0.1), 0.01, shader, V3D(0.0, 1.0, 0.0));
    drawArrow3d(rb->getWorldCoordinates(P3D()), V3D(rb->getWorldCoordinates(V3D(0, 0, 1)) * 0.1), 0.01, shader, V3D(0.0, 0.0, 1.0));
}

void RBRenderer::drawSkeletonView(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, bool showJointAxes, bool showJointLimits,
                                  bool showJointAngles, float alpha) {
    // draw capsules that define the "skeleton" of this body: parent joints TO
    // features TO child joints and end effectors

    V3D drawColor = rb->rbProps.selected ? rb->rbProps.highlightColor : rb->rbProps.color;

    if (rb->pJoint != NULL) {
        P3D startPos = rb->getWorldCoordinates(rb->pJoint->cJPos);
        P3D endPos = rb->getWorldCoordinates(P3D(0, 0, 0));
        drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor, alpha);
    }

    for (uint i = 0; i < rb->cJoints.size(); i++) {
        P3D startPos = rb->getWorldCoordinates(P3D(0, 0, 0));
        P3D endPos = rb->getWorldCoordinates(rb->cJoints[i]->pJPos);
        drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor, alpha);
    }

    if (rb->cJoints.size() == 0) {
        for (uint i = 0; i < rb->rbProps.endEffectorPoints.size(); i++) {
            P3D startPos = rb->getWorldCoordinates(P3D(0, 0, 0));
            P3D endPos = rb->getWorldCoordinates(rb->rbProps.endEffectorPoints[i].endEffectorOffset);
            drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor, alpha);
        }
    }

    for (uint i = 0; i < rb->cJoints.size(); i++) {
        V3D drawColor = rb->cJoints[i]->selected ? rb->rbProps.highlightColor : rb->rbProps.jointDrawColor;

        P3D globalJointPos = rb->cJoints[i]->getWorldPosition();
        drawSphere(globalJointPos, rb->rbProps.abstractViewCylRadius * 1.3, shader, drawColor, alpha);
    }

    // draw joint limits
    if (showJointLimits) {
        for (uint i = 0; i < rb->cJoints.size(); i++) {
            drawJointLimits(rb->cJoints[i], shader);
        }
    }

    // drawing joints happens in world coordinates directly...
    if (showJointAxes) {
        for (uint i = 0; i < rb->cJoints.size(); i++)
            drawAxis(rb->cJoints[i], shader);
    }

    if (showJointAngles) {
        for (uint i = 0; i < rb->cJoints.size(); i++)
            drawJointAngle(rb->cJoints[i], shader);
    }
}

void RBRenderer::drawMeshes(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, float alpha) {
    for (auto &m : rb->rbProps.models) {
        RigidTransformation meshTransform(rb->getOrientation(), rb->getWorldCoordinates(P3D()));
        meshTransform *= m.localT;

        m.position = meshTransform.T;
        m.orientation = meshTransform.R;

        if (rb->rbProps.selected)
            m.draw(shader, rb->rbProps.highlightColor, alpha, true);
        else
            m.draw(shader, m.color, alpha, true);
    }
}

void RBRenderer::drawCollisionShapes(const std::shared_ptr<const RB> &rb, const gui::Shader &shader) {
    for (uint i = 0; i < rb->rbProps.collisionShapes.size(); i++) {
        if (const auto &cs = std::dynamic_pointer_cast<RRBCollisionSphere>(rb->rbProps.collisionShapes[i])) {
            P3D pos = rb->getWorldCoordinates(cs->localCoordinates);
            drawSphere(pos, cs->radius, shader, rb->rbProps.colSphereDrawColor);
        }

        if (const auto &cb = std::dynamic_pointer_cast<RRBCollisionBox>(rb->rbProps.collisionShapes[i])) {
            P3D pos = rb->getWorldCoordinates(cb->localCoordinates);
            Quaternion quat = rb->getOrientation() * cb->localOrientation;
            drawCuboid(pos, quat, cb->dimensions, shader, rb->rbProps.colSphereDrawColor);
        }

        if (const auto &cyl = std::dynamic_pointer_cast<RRBCollisionCylinder>(rb->rbProps.collisionShapes[i])) {
            P3D start = rb->getWorldCoordinates(cyl->localCoordinates + cyl->localOrientation * V3D(0, 0, -0.5 * cyl->length));
            P3D end = rb->getWorldCoordinates(cyl->localCoordinates + cyl->localOrientation * V3D(0, 0, 0.5 * cyl->length));
            drawCylinder(start, end, cyl->radius, shader, rb->rbProps.colSphereDrawColor);
        }

        if (const auto &cap = std::dynamic_pointer_cast<RRBCollisionCapsule>(rb->rbProps.collisionShapes[i])) {
            P3D start = rb->getWorldCoordinates(cap->localCoordinates + cap->localOrientation * V3D(0, 0, -0.5 * cap->length));
            P3D end = rb->getWorldCoordinates(cap->localCoordinates + cap->localOrientation * V3D(0, 0, 0.5 * cap->length));
            drawCapsule(start, end, cap->radius, shader, rb->rbProps.colSphereDrawColor);
        }
    }
}

void RBRenderer::drawMOI(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, bool wireFrame) {
    Eigen::EigenSolver<Matrix3x3> eigenvalueSolver(rb->rbProps.MOI_local);

    Eigen::Vector3cd principleMomentsOfInertia = eigenvalueSolver.eigenvalues();

    assert(IS_ZERO(principleMomentsOfInertia[0].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()));

    Eigen::Matrix3cd V = eigenvalueSolver.eigenvectors();

    double Ixx = principleMomentsOfInertia[0].real();  // = m(y2 + z2)/12
    double Iyy = principleMomentsOfInertia[1].real();  // = m(z2 + x2)/12
    double Izz = principleMomentsOfInertia[2].real();  // = m(y2 + x2)/12

    double x = sqrt((Iyy + Izz - Ixx) * 6 / rb->rbProps.mass);
    double y = sqrt((Izz + Ixx - Iyy) * 6 / rb->rbProps.mass);
    double z = sqrt((Ixx + Iyy - Izz) * 6 / rb->rbProps.mass);

    P3D pmin(-x / 2, -y / 2, -z / 2), pmax(x / 2, y / 2, z / 2);

    if (V.determinant().real() < 0.0) {
        V(0, 2) *= -1;
        V(1, 2) *= -1;
        V(2, 2) *= -1;
    }
    assert(IS_ZERO(abs(V.determinant().real() - 1.0)) && "Rotation matrices have a determinant which is equal to 1.0!");

    Quaternion q(V.real());

    if (wireFrame == false)
        drawCuboid(rb->getWorldCoordinates(P3D()), rb->getOrientation() * q, V3D(x, y, z), shader, V3D(0.7, 0.7, 0.7));
    else
        drawWireFrameCuboid(rb->getWorldCoordinates(P3D()), rb->getOrientation() * q, V3D(x, y, z), shader, V3D(0.7, 0.7, 0.7));
}

void RBRenderer::drawEndEffectors(const std::shared_ptr<const RB> &rb, const gui::Shader &shader) {
    for (uint i = 0; i < rb->rbProps.endEffectorPoints.size(); i++) {
        auto &ee = rb->rbProps.endEffectorPoints[i];
        P3D pos = rb->getWorldCoordinates(ee.endEffectorOffset);
        V3D color = ee.inContact ? rb->rbProps.contactedEndEffectorDrawColor : rb->rbProps.endEffectorDrawColor;
        drawSphere(pos, rb->rbProps.endEffectorRadius, shader, color);
    }
}

void RBRenderer::drawAxis(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader) {
    drawArrow3d(j->getWorldPosition(), V3D(j->parent->getWorldCoordinates(j->rotationAxis) * 0.1), 0.01, shader, V3D(1.0, 0.0, 0.0));
}

void RBRenderer::drawJointLimits(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader) {
    if (!j->jointAngleLimitsActive)
        return;

    P3D p = j->getWorldPosition();
    V3D n = j->parent->getWorldCoordinates(j->rotationAxis);

    // this is the feature that we will be tracking as the joint rotates through
    // its range of motion... expressed in the coordinate frame of the child, as
    // it is the thing that rotates with the joint...
    V3D vFeature = V3D(j->cJPos, P3D());
    if (j->child->cJoints.size() == 1)
        vFeature = V3D(j->cJPos, j->child->cJoints[0]->pJPos);

    vFeature -= j->rotationAxis.dot(vFeature) * j->rotationAxis;

    if (vFeature.norm() < 0.000001)
        return;

    // now, we need to see what this feature vector would look like when rotated
    // according to the range of motion of the joint...
    V3D from = j->parent->getWorldCoordinates(getRotationQuaternion(j->minAngle, j->rotationAxis) * vFeature);
    V3D to = j->parent->getWorldCoordinates(getRotationQuaternion(j->maxAngle, j->rotationAxis) * vFeature);
    V3D currentV = j->child->getWorldCoordinates(vFeature);

    drawSector(p, from, to, n, shader, V3D(1, 0, 0));
    drawArrow3d(p, currentV, 0.01, shader, V3D(0, 1, 0));
}

void RBRenderer::drawJointAngle(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader) {
    P3D p = j->getWorldPosition();
    V3D n = j->parent->getWorldCoordinates(j->rotationAxis);

    // this is the feature that we will be tracking as the joint rotates through
    // its range of motion... expressed in the coordinate frame of the child, as
    // it is the thing that rotates with the joint...
    V3D vFeature = V3D(j->cJPos, P3D());
    if (j->child->cJoints.size() == 1)
        vFeature = V3D(j->cJPos, j->child->cJoints[0]->pJPos);

    vFeature -= j->rotationAxis.dot(vFeature) * j->rotationAxis;

    if (vFeature.norm() < 0.000001)
        return;

    // now, we need to see what this feature vector would look like when rotated
    // according to the range of motion of the joint...
    V3D from = j->parent->getWorldCoordinates(getRotationQuaternion(0, j->rotationAxis) * vFeature);
    V3D to = j->child->getWorldCoordinates(vFeature);

    drawSector(p, from, to, n, shader, V3D(1, 0, 0));
}

}  // namespace crl::loco