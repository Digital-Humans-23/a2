#include "loco/robot/RBEngine.h"

namespace crl::loco {

void RBEngine::addRobotToEngine(const std::shared_ptr<Robot> &robot) {
    robots.push_back(robot);
}

void RBEngine::addRigidBodyToEngine(const std::shared_ptr<RB> &rb) {
    rbs.push_back(rb);
}

void RBEngine::addJointToEngine(const std::shared_ptr<RBJoint> &j) {
    joints.push_back(j);
}

void RBEngine::loadRBsFromFile(const char *fName) {
    RBLoader rbLoader(fName);
    rbLoader.populateRBEngine(*this);
}

void RBEngine::markEEContacts(double threshold) {
    for (auto rb : this->rbs) {
        // as a default, check if ee is under the ground (height < 0)
        for (auto &ee : rb->rbProps.endEffectorPoints) {
            ee.inContact = (rb->getWorldCoordinates(ee.endEffectorOffset).y - ee.radius) < threshold;
        }
    }
}

}  // namespace crl::loco