//
// Created by Dongho Kang on 06.09.22.
//

#ifndef PYLOCO_VIEWER_H
#define PYLOCO_VIEWER_H

#include "crl-basic/gui/application.h"
#include "pylocobase/sim/Simulator.h"

namespace pyloco {

class Viewer : public crl::gui::ShadowApplication {
public:
    /**
     * create a full-screen viewer
     */
    explicit Viewer(Simulator *sim) : crl::gui::ShadowApplication("Pyloco Viewer"), sim_(sim) {
        init();
    }

    /**
     * create a viewer with specified width and height
     */
    explicit Viewer(Simulator *sim, int width, int height) : crl::gui::ShadowApplication("Pyloco Viewer", width, height), sim_(sim) {
        init();
    }

    ~Viewer() override = default;

    void drawImGui() override {
//        ImGui::Begin("Main Menu", NULL, ImGuiWindowFlags_AlwaysAutoResize);
//        ImGui::Checkbox("Show Main Menu", &showMainMenu);
//        ImGui::End();
//
//        if (showMainMenu) {
//            crl::gui::ShadowApplication::drawImGui();
//            ImGui::Begin("Main Menu");
//            sim_->drawImGui();
//            ImGui::End();
//        }
    }

    void drawImPlot() override {
        ShadowApplication::drawImPlot();

        ImGui::Begin("Plots");
        sim_->drawImPlot();
        ImGui::End();
    }

    void render() {
        process();
        draw();

#ifdef SINGLE_BUFFER
        glFlush();
#else
        glfwSwapBuffers(window);
#endif
        glfwPollEvents();

        const auto &center = robot_->getTrunk()->getWorldCoordinates(crl::P3D());
        camera.target.x = (float)center.x;
        camera.target.z = (float)center.z;
    }

    void readPixels(unsigned char* rgb) {
        using namespace crl::gui;
        GLCall(glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb));
    }

    void close() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        crl::gui::rendering::DestroyContext();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
    }

private:
    /**
     * initialization
     */
    void init() {
        this->showConsole = false;
        this->automanageConsole = true;
        this->useSeparateProcessThread = false;

        // setup robot (graphics instance)
        robot_ = std::make_shared<crl::loco::LeggedRobot>(sim_->rbsFilePath_.c_str());
        robot_->showSkeleton = true;

        RobotInfo::Model robotModel = sim_->getRobotModel();
        switch (robotModel) {
            case RobotInfo::Model::Bob: {
                camera.target.y =sim_->robot_->getRoot()->getState().pos.y;
                camera.distanceToTarget = 5.0;
                camera.rotAboutUpAxis = PI * 0.25;
                camera.rotAboutRightAxis = PI * 0.1;
            } break;
            case RobotInfo::Model::Dog:
            case RobotInfo::Model::Go1:
            default: {
                camera.distanceToTarget = 2.5;
                camera.rotAboutUpAxis = PI * 0.25;
                camera.rotAboutRightAxis = PI * 0.1;
            } break;
        }

        // disable waiting for framerate of glfw window
        glfwSwapInterval(0);
    }

    /**
     * the purpose of viewer is only show robot's current state. thus, we hide process function so that no one can change this function.
     */
    void process() override {
        crl::dVector q = sim_->getQ();
        crl::dVector qDot = sim_->getQDot();
        crl::loco::GCRR gcrr(robot_);
        gcrr.setQ(q);
        gcrr.setQDot(qDot);
        gcrr.syncRobotStateWithGeneralizedCoordinates();
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        robot_->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        robot_->draw(shader);
    }

private:
    Simulator *sim_;
    std::shared_ptr<crl::loco::LeggedRobot> robot_ = nullptr;
    bool showMainMenu = false;
    RobotInfo::Model model;
};

}  // namespace pyloco

#endif  //PYLOCO_VIEWER_H
