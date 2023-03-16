//
// Created by Dongho Kang on 11.12.21.
//
#include "loco/LeggedRobot.h"

namespace crl::loco {

LeggedRobot::LeggedRobot(const char *filePath, const char *statePath, bool loadVisuals)
    : Robot(filePath, statePath, loadVisuals), standingState(*this), trunk(root) {}

std::shared_ptr<RB> LeggedRobot::getTrunk() {
    return trunk;
}

void LeggedRobot::addLimb(const std::string &name, const std::string &eeRBName) {
    addLimb(name, getRBByName(eeRBName.c_str()));
}

void LeggedRobot::addLimb(const std::string &name, const std::shared_ptr<RB> &eeRB) {
    limbs.push_back(std::make_shared<RobotLimb>(name, eeRB, trunk));
    limbs.back()->limbIndex = limbs.size() - 1;
}

std::shared_ptr<RobotLimb> LeggedRobot::getLimbByName(const std::string &name) const {
    for (auto limbPtr : limbs) {
        if (limbPtr->name == name) {
            return limbPtr;
        }
    }
    // No limb matched the name, so return a null pointer
    return nullptr;
}

int LeggedRobot::getLimbCount() const {
    return limbs.size();
}

std::shared_ptr<RobotLimb> LeggedRobot::getLimb(uint i) const {
    if (i < limbs.size())
        return limbs[i];
    return nullptr;
}

}  // namespace crl::loco