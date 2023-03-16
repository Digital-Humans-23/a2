#ifndef PYLOCO_SIMULATOR_H
#define PYLOCO_SIMULATOR_H

#include <loco/LeggedRobot.h>
#include <loco/robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <loco/robot/RobotState.h>
#include <loco/simulation/ode/ODERBEngine.h>

#include "pylocobase/RobotInfo.h"

namespace pyloco {

/**
 * Base simulator class for pyloco projects. It loads quadrupedal model into ODE physics simulator.
 */
class Simulator {
public:
    std::shared_ptr<crl::loco::ODERBEngine> rbEngine_ = nullptr;
    std::shared_ptr<crl::loco::LeggedRobot> robot_ = nullptr;

    // constant
    const double simTimeStepSize = 1.0 / 120;
    const double controlTimeStepSize = 1.0 / 60;

    // parameters
    double motorsKp = 300;
    double motorsKd = 10;
    double motorMaxTorque = 55.0;

    // properties
    // these are populated when the simulator is initialized
    std::string rbsFilePath_;
    crl::dVector maxJointAngle_;
    crl::dVector minJointAngle_;
    crl::dVector nominalJointAngle_;
    crl::dVector zeroJointAngle_;
    crl::dVector jointScaleFactors_;
    double nominalBaseHeight_ = 0.5;
    uint numJoints_ = 0;

    crl::dVector allLoopMotorTorques;
    bool lockSelectedJoints = false;
    double box_speed = 2.1;

protected:
    double timeStamp_ = 0;
    int throwCounter = 0;
    std::shared_ptr<crl::loco::RB> box0 = std::make_shared<crl::loco::RB>();
    std::shared_ptr<crl::loco::RB> box1 = std::make_shared<crl::loco::RB>();
    std::shared_ptr<crl::loco::RB> box2 = std::make_shared<crl::loco::RB>();

private:
    RobotInfo::Model robotModel = RobotInfo::Model::Dog;

public:
    Simulator(double simTimeStepSize, double controlTimeStepSize, uint robotModel = (uint)RobotInfo::Model::Dog, bool loadVisuals = false);

    virtual ~Simulator() = default;

    /**
     * you can implement observation getter in c++ side.
     * if the argument reduced is true, it removes x,z and yaw observation.
     */
    virtual crl::dVector getObservation(bool reduced) const;

    /**
     * you can implement action scaling scheme in c++ side.
     */
    virtual crl::dVector getScaledAction(const crl::dVector &action) const;

    /**
     * implement reset logic for your simulator. this is pure virtual function but it has a default implementation.
     * in the base class implement override function as follows:
     *
     * DerivedSimulator::reset() override {
     *     Simulator()::reset();    // call default implementation first.
     *     ...                      // some additional reset logic required in the derived class
     * }
     */
    virtual void reset() = 0;

    /**
     * reset with given initial configuration
     */
    void reset(const crl::dVector &q, const crl::dVector &qDot);

    /**
     * set generalized coordinates and generalized velocity of robot.
     */
    void setQAndQDot(const crl::dVector &q, const crl::dVector &qDot);

    /**
     * environment step. apply control and call simulation steps. implement logic in derived class.
     */
    virtual void step(const crl::dVector &jointTarget) = 0;

    /**
     * get simulation time since it's been started (or restarted)
     */
    double getTimeStamp() const {
        return timeStamp_;
    }

    /**
     * get generalized coordinates of robot configuration
     */
    crl::dVector getQ() const;

    /**
     * get generalized velocity of robot configuration
     */
    crl::dVector getQDot() const;

    /**
     * get nun_feet x 3 matrix which contains feet position
     */
    crl::Matrix getFeetPosition() const;

    /**
     * get nun_feet x 3 matrix which contains feet velocity
     */
    crl::Matrix getFeetVelocity() const;

    /**
     * check if the robot collapsed by checking if collision happens in any rigid body other than feet.
     */
    bool isRobotCollapsed() const;

    /**
     * get contact state of feet
     */
    crl::dVector getFeetContact() const;

    /**
     * get motor torques
     */
    crl::dVector getMotorTorques() const;

    /**
     * TODO: get all loop motor torques
     */
    crl::dVector getAllLoopMotorTorques() const;

    /**
     * Throw box with a given strength
     */
    void throwBox(int boxIdx, double strength, crl::dVector initialPos);


    /**
     * Get robot model
     */
    RobotInfo::Model getRobotModel(){
        return robotModel;
    }


    /**
     * --------------------------------------------------------------------------------------------
     * these APIs for debugging in C++. do not bind these for python.
     * --------------------------------------------------------------------------------------------
     */

    /**
     * render information useful for debugging.
     */
    virtual void drawDebugInfo(const crl::gui::Shader &shader) const {};

    /**
     * draw options on ImGui.
     */
    virtual void drawImGui();

    /**
     * draw plot using ImPlot.
     */
    virtual void drawImPlot() const {};

protected:
    void resetToDefaultRobotState();

    /**
     * set joint target
     */
    void applyControlSignal(const crl::dVector &jointTarget);

    /**
     * one step of simulation with timestep size dt
     */
    void simulationStep();

    /**
     * setup box to throw
     */
    void setupBox(const std::shared_ptr<crl::loco::RB>&dynamicBox, std::string name);


};

}  // namespace pyloco

#endif  //PYLOCO_SIMULATOR_H