#pragma once

#include <crl-basic/gui/model.h>
#include <crl-basic/gui/shader.h>

#include "loco/robot/RBJoint.h"

namespace crl::loco {

class RBRenderer {
public:
    /* methods used to draw different types of views of the rigid body... */
    static void drawSkeletonView(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, bool showJointAxes, bool showJointLimits, bool showJointAngle,
                                 float alpha = 1.0);

    static void drawMeshes(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, float alpha = 1.0);

    static void drawCoordFrame(const std::shared_ptr<const RB> &rb, const gui::Shader &shader);

    static void drawCollisionShapes(const std::shared_ptr<const RB> &rb, const gui::Shader &shader);

    static void drawMOI(const std::shared_ptr<const RB> &rb, const gui::Shader &shader, bool wireFrame = false);

    static void drawEndEffectors(const std::shared_ptr<const RB> &rb, const gui::Shader &shader);

    /**
     * draws the axes of rotation
     */
    static void drawAxis(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader);

    /**
     * draw joint limits
     */
    static void drawJointLimits(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader);

    /**
     * draw joint angle
     */
    static void drawJointAngle(const std::shared_ptr<const RBJoint> &j, const gui::Shader &shader);
};

}  // namespace crl::loco