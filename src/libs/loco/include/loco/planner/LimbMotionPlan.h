#ifndef CRL_LOCO_LIMBMOTIONPLAN_H
#define CRL_LOCO_LIMBMOTIONPLAN_H

#include <crl-basic/gui/renderer.h>

#include "loco/gait/ContactSchedule.h"
#include "loco/utils/TrajectoryUtils.h"

namespace crl::loco {

class LimbMotionProperties {
public:
    double contactSafetyFactor = 0.7;
    double ffStancePhaseForDefaultStepLength = 0.5;
    double swingFootHeight = 0.1;

    //x and z here are expressed in a heading-independent coordinate frame
    double stepWidthOffsetX = 0.7;
    double stepWidthOffsetZ = 1.0;

    LimbMotionProperties() = default;
};

class LimbContactPhase {
public:
    // the time the contact phase starts
    double tStart = 0;
    // the time it ends
    double tEnd = 0;
    // and the position where the limb is/will be in contact with the environment
    P3D contactLocation;
    // this is the ideal contact location, according to whatever model is used to
    // generate footsteps...
    P3D idealFootstepLocation;
    // we will also add a flag here that tells us if this is a current position (i.e. fixed) in execution or already executed, or a planned one
    bool isFixed = false;
    LimbContactPhase(double tStart, double tEnd, const P3D& pos, bool isFixed = false) {
        this->tStart = tStart;
        this->tEnd = tEnd;
        this->idealFootstepLocation = this->contactLocation = pos;
        this->isFixed = isFixed;
    }
};

/**
 * This class stores a sequence of planned foot falls for each of the robot's limbs.
 * Useful bits of information, such as the target stepping location at a particular moment in time can be easily retrieved.
 */
class LimbMotionPlan {
public:
    // planner should populate these before generating plan
    LimbMotionProperties lmp;
    ContactSchedule cs;
    std::map<std::string, P3D> eePos;  // for very first swing

    // this is a pointer to the contact plan manager which corresponds to the sequnce of foot steps that is planned here. Whomever populates the footstep plan
    // should also set the contact plan manager adequately; the two are closely link, as they store complementary information
    std::map<std::string, DynamicArray<LimbContactPhase>> footSteps;

public:
    /**
     * erase footsteps that ends < t.
     */
    void cleanUpBeforeT(const std::string& limb, double t) {
        auto& fss = footSteps[limb];
        while (!fss.empty() && fss[0].tEnd < t)
            fss.erase(fss.begin());
    }

    P3D getBezierEEPositionAtTime(const std::string& limb, double t) {
        ContactPhaseInfo cp = cs.getContactPhaseInformation(limb, t);
        if (cp.isStance()) {
            P3D contactPos = getPlannedContactLocationAt_t(limb, t);
            return contactPos;
        }
        P3D takeoff = getPlannedContactLocationBefore_t(limb, t);
        P3D landing = getPlannedContactLocationAfter_t(limb, t);
        return evaluateBezierPosition(takeoff, landing, lmp.swingFootHeight, cp.getPercentageOfTimeElapsed());
    }

    V3D getBezierEEVelocityAtTime(const std::string& limb, double t) {
        ContactPhaseInfo cp = cs.getContactPhaseInformation(limb, t);
        if (cp.isStance()) {
            return {0, 0, 0};
        }
        P3D takeoff = getPlannedContactLocationBefore_t(limb, t);
        P3D landing = getPlannedContactLocationAfter_t(limb, t);
        return evaluateBezierVelocity(takeoff, landing, lmp.swingFootHeight, cp.getPercentageOfTimeElapsed(), cp.getDuration());
    }

    V3D getBezierEEAccelerationAtTime(const std::string& limb, double t) {
        ContactPhaseInfo cp = cs.getContactPhaseInformation(limb, t);
        if (cp.isStance()) {
            return {0, 0, 0};
        }
        P3D takeoff = getPlannedContactLocationBefore_t(limb, t);
        P3D landing = getPlannedContactLocationAfter_t(limb, t);
        return evaluateBezierAcceleration(takeoff, landing, lmp.swingFootHeight, cp.getPercentageOfTimeElapsed(), cp.getDuration());
    }

    LimbContactPhase* getPlannedLimbContactAt_t(const std::string& limb, double t) {
        int idx = getIndexOfPlannedLimbContactAt_t(limb, t);
        if (idx == -1)
            return nullptr;
        return &footSteps[limb][idx];
    }

    LimbContactPhase* getPlannedLimbContactAfter_t(const std::string& limb, double t) {
        int idx = getIndexOfPlannedLimbContactAfter_t(limb, t);
        if (idx == -1)
            return nullptr;
        return &footSteps[limb][idx];
    }

    LimbContactPhase* getPlannedLimbContactBefore_t(const std::string& limb, double t) {
        int idx = getIndexOfPlannedLimbContactBefore_t(limb, t);
        if (idx == -1)
            return nullptr;
        return &footSteps[limb][idx];
    }

private:
    int getIndexOfPlannedLimbContactBefore_t(const std::string& limb, double t) {
        for (int i = footSteps[limb].size() - 1; i >= 0; i--) {
            // note. this function is called when limb is in swing at t.
            // if footstep.tEnd <= t, we consider the footstep is about to be finished.
            if (footSteps[limb][i].tEnd <= t)
                return i;
        }
        return -1;
    }

    P3D getPlannedContactLocationBefore_t(const std::string& limb, double t) {
        const LimbContactPhase* lcp = getPlannedLimbContactBefore_t(limb, t);
        if (!lcp)
            return eePos[limb];
        return lcp->contactLocation;
    }

    int getIndexOfPlannedLimbContactAt_t(const std::string& limb, double t) {
        for (int i = 0; i < footSteps[limb].size(); i++) {
            if (footSteps[limb][i].tStart <= t && t <= footSteps[limb][i].tEnd)
                return i;
        }
        return -1;
    }

    P3D getPlannedContactLocationAt_t(const std::string& limb, double t) {
        const LimbContactPhase* lcp = getPlannedLimbContactAt_t(limb, t);
        if (!lcp)
            return eePos[limb];
        return lcp->contactLocation;
    }

    int getIndexOfPlannedLimbContactAfter_t(const std::string& limb, double t) {
        for (int i = 0; i < footSteps[limb].size(); i++) {
            // note. this function is called when limb is in swing at t.
            // if footstep.tStart >= t, we consider the footstep is about to be executed
            if (footSteps[limb][i].tStart >= t)
                return i;
        }
        return -1;
    }

    P3D getPlannedContactLocationAfter_t(const std::string& limb, double t) {
        const LimbContactPhase* lcp = getPlannedLimbContactAfter_t(limb, t);
        if (!lcp)  // this should not happen... but just in case.
            return eePos[limb];
        return lcp->contactLocation;
    }
};

}  // namespace crl::loco

#endif  //CRL_LOCO_LIMBMOTIONPLAN_H