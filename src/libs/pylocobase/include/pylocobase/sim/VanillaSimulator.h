//
// Created by Dongho Kang on 10.09.22.
//

#ifndef PYLOCO_VANILLASIMULATOR_H
#define PYLOCO_VANILLASIMULATOR_H

#include "crl-basic/gui/plots.h"
#include "pylocobase/sim/Simulator.h"

namespace pyloco {

/**
 * Simulator for Vanilla policy which generates joint-level P targets from user's high-level command.
 */
class VanillaSimulator : public Simulator {
public:
    double commandForwardSpeed = 1.0;

private:
    // plots
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> baseSpeedPlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> jointAnglePlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> jointSpeedPlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> jointAccelerationPlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> jointTorquePlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> jointActionPlot = nullptr;
    std::shared_ptr<crl::gui::RealTimeLinePlot2D<crl::dVector>> outputTorquePlot = nullptr;


public:
    explicit VanillaSimulator(double simTimeStepSize,
                     double controlTimeStepSize,
                     uint robotModel = (uint)RobotInfo::Model::Dog, bool loadVisuals = false) : Simulator(simTimeStepSize, controlTimeStepSize, robotModel, loadVisuals) {
        baseSpeedPlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Base Velocity", "[sec]", "[m/s]");
        jointAnglePlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Joint Angle", "[sec]", "[rad]");
        jointSpeedPlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Joint Speed", "[sec]", "[rad/s]");
        jointAccelerationPlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Joint Acceleration", "[sec]", "[rad/s^2]");
        jointTorquePlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Joint Torque", "[sec]", "[N.m]");
        jointActionPlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Joint Action Angle", "[sec]", "[rad]");
        outputTorquePlot = std::make_shared<crl::gui::RealTimeLinePlot2D<crl::dVector>>("Output Joint Torque", "[sec]", "[N.m]");

        for (int i = 0; i < robot_->getJointCount(); i++) {
            jointAnglePlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
            jointSpeedPlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
            jointAccelerationPlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
            jointTorquePlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
            jointActionPlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
            outputTorquePlot->addLineSpec({robot_->getJoint(i)->name, [i](const auto &d) { return (float)d[i]; }});
        }
        baseSpeedPlot->addLineSpec({"Base Velocity", [](const auto &d) { return (float)d[0]; }});
        baseSpeedPlot->addLineSpec({"Target Velocity", [](const auto &d) { return (float)d[1]; }});
        allLoopMotorTorques.resize(robot_->getJointCount() * int(controlTimeStepSize / simTimeStepSize + 1.0e-10));
    }

    ~VanillaSimulator() override = default;

    void reset() override {
        Simulator::reset();
        robot_->setRootState(crl::P3D(0, nominalBaseHeight_, 0), crl::Quaternion::Identity());
        baseSpeedPlot->clearData();
        jointAnglePlot->clearData();
        jointSpeedPlot->clearData();
        jointAccelerationPlot->clearData();
        jointTorquePlot->clearData();
        jointActionPlot->clearData();
        outputTorquePlot->clearData();
        allLoopMotorTorques.setZero();
    }

    void step(const crl::dVector &jointTarget) override {
        crl::dVector initial_joint_angles = getQ().tail(numJoints_);
        allLoopMotorTorques.setZero();  // set to zero

        applyControlSignal(jointTarget);
        jointActionPlot->addData((float)getTimeStamp(), jointTarget);


        double simTime = 0;
        int numSimStepsPerLoop = (int)((controlTimeStepSize + 1e-10) / simTimeStepSize);

        for (int i = 0; i < numSimStepsPerLoop; i++) {
            crl::dVector PastJointSpeed = getQDot().tail(numJoints_);

            // step the simulation
            simulationStep();

            allLoopMotorTorques.segment(i * numJoints_, numJoints_) = getMotorTorques();

            // populate plots
            jointAnglePlot->addData((float)getTimeStamp(), getQ().tail(numJoints_));
            jointSpeedPlot->addData((float)getTimeStamp(), getQDot().tail(numJoints_));
            jointTorquePlot->addData((float)getTimeStamp(), getMotorTorques());
            crl::dVector JointAcceleration = (getQDot().tail(numJoints_) - PastJointSpeed) / simTimeStepSize;
            jointAccelerationPlot->addData((float)getTimeStamp(), JointAcceleration);
            crl::dVector baseSpeeds(2);
            baseSpeeds[0] = robot_->getRoot()->getLocalCoordinates(robot_->getRoot()->getVelocityForPoint_local(crl::P3D())).z();
            baseSpeeds[1] = commandForwardSpeed;
            baseSpeedPlot->addData((float)getTimeStamp(), baseSpeeds);

            // advance in time
            simTime += simTimeStepSize;
        }
        //        outputTorquePlot->addData((float)getTimeStamp(), getMotorTorques());
    }

    void drawImPlot() const override {
        jointAnglePlot->draw();
        jointActionPlot->draw();
        jointSpeedPlot->draw();
        jointAccelerationPlot->draw();
        jointTorquePlot->draw();
        baseSpeedPlot->draw();
        //        outputTorquePlot->draw();
    }

    crl::dVector getAllLoopMotorTorques() const {
        return allLoopMotorTorques;
    }
};

}  // namespace pyloco

#endif  //PYLOCO_VANILLASIMULATOR_H
