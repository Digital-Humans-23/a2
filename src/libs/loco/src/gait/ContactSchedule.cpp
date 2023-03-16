//
// Created by Dongho Kang on 18.12.21.
//
#include "loco/gait/ContactSchedule.h"

namespace crl::loco {

// use Dear ImGUI code to visualize the contact schedule...
void ContactPlanManager::visualizeContactSchedule(double t, double gridStartTime, double dt_grid, int nGridSteps) const {
    ImGui::SetNextWindowBgAlpha(1.0);
    ImGui::Begin("Contact Schedule Visualizer");
    ImGuizmo::BeginFrame();
    ImGui::Checkbox("draw labels", &drawLabels);

    // this is where the window screen starts...
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    double timeStart = t - visTimeWindow * 0.25;
    double timeEnd = t + visTimeWindow;

    // this is for every row which will visualize the foot fall pattern for
    // one robot limb...
    float height = ImGui::GetFrameHeight();
    float radius = height * 0.005f;
    ImU32 col_bg = ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
    draw_list->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + labelWindowWidth + cpmWindowWidth + 20, p.y + cs.swings.size() * height), col_bg);

    ImU32 col_text_gray = ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 1.0f, 1.0f));
    ImU32 col_line = ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
    ImU32 col_cursor = ImGui::GetColorU32(ImVec4(1.0f, 0.7f, 0.7f, 1.0f));
    ImU32 col_horizon = ImGui::GetColorU32(ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
    ImU32 col_text_dark = ImGui::GetColorU32(ImVec4(0.1f, 0.1f, 0.1f, 1.0f));

    // draw the swing phases now...
    float rowOffset = 0;
    for (const auto &lcs : cs.swings) {
        ContactPhaseInfo cpi = cs.getContactPhaseInformation(lcs.limbName, t);
        for (auto sp : lcs.swingPhases) {
            float start = getWindowCoord(sp.first, timeStart, timeEnd);
            float end = getWindowCoord(sp.second, timeStart, timeEnd);
            if (end > labelWindowWidth && start < labelWindowWidth + cpmWindowWidth) {
                ImU32 col = ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
                if (start < labelWindowWidth - 5)
                    start = labelWindowWidth - 5;
                if (end > labelWindowWidth + cpmWindowWidth + 5)
                    end = labelWindowWidth + cpmWindowWidth + 5;
                // only mark as in swing the relevant one...
                if (cpi.isSwing() && sp.first <= t && sp.second >= t)
                    col = ImGui::GetColorU32(ImVec4(1.0f - 0.3 * cpi.getPercentageOfTimeElapsed(), 0.7f * (cpi.getPercentageOfTimeElapsed()),
                                                    0.7f * (cpi.getPercentageOfTimeElapsed()), 1.0f));
                draw_list->AddRectFilled(ImVec2(p.x + start, p.y + rowOffset + height * 0.15), ImVec2(p.x + end, p.y + rowOffset + height * 0.85), col,
                                         height * radius);
            }
        }
        rowOffset += height;
    }

    // cover up the loose ends of the swing phases...

    draw_list->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + labelWindowWidth, p.y + rowOffset), col_bg);
    draw_list->AddRectFilled(ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y), ImVec2(p.x + labelWindowWidth + cpmWindowWidth + 20, p.y + rowOffset),
                             col_bg);

    // draw the grid, the labels of the limbs, and the time cursor...
    rowOffset = 0;
    draw_list->AddLine(ImVec2(p.x, p.y + rowOffset), ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y + rowOffset), col_line);
    for (const auto &lcs : cs.swings) {
        draw_list->AddText(ImVec2(p.x + height * 0.5, p.y + rowOffset + height * 0.1), col_text_gray, lcs.limbName.c_str());
        rowOffset += height;
        draw_list->AddLine(ImVec2(p.x, p.y + rowOffset), ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y + rowOffset), col_line);
    }
    draw_list->AddLine(ImVec2(p.x, p.y), ImVec2(p.x, p.y + rowOffset), col_line);
    draw_list->AddLine(ImVec2(p.x + labelWindowWidth, p.y), ImVec2(p.x + labelWindowWidth, p.y + rowOffset), col_line);
    draw_list->AddLine(ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y), ImVec2(p.x + labelWindowWidth + cpmWindowWidth, p.y + rowOffset), col_line);

    for (auto s : strideUpdates) {
        float sVal = getWindowCoord(s.first, timeStart, timeEnd);
        if (sVal >= labelWindowWidth && sVal <= labelWindowWidth + cpmWindowWidth)
            draw_list->AddLine(ImVec2(p.x + sVal, p.y - 5), ImVec2(p.x + sVal, p.y + rowOffset + 5), col_horizon);
    }

    // show where the current moment in time is...
    float cursorVal;

    // TODO: for mpc only
    if (gridStartTime > -1) {
        int N = nGridSteps;
        for (int i = 0; i < N + 1; i++) {
            double t_tmp = gridStartTime + i * dt_grid;
            if (t_tmp >= timeStart) {
                cursorVal = getWindowCoord(t_tmp, timeStart, timeEnd);

                if (i == 0 || i == N) {
                    draw_list->AddLine(ImVec2(p.x + cursorVal, p.y), ImVec2(p.x + cursorVal, p.y + rowOffset), col_line, 2);
                    draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y - 5), 5, col_line);
                    draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y + rowOffset + 5), 5, col_line);
                    char timeTextForMPC[100];
                    sprintf(timeTextForMPC, "t = %2.2lfs", t_tmp);
                    draw_list->AddText(ImVec2(p.x + cursorVal - height / 2.0, p.y + rowOffset + height * 0.5), col_text_gray, timeTextForMPC);
                } else {
                    draw_list->AddLine(ImVec2(p.x + cursorVal, p.y), ImVec2(p.x + cursorVal, p.y + rowOffset), col_line, 0.5);
                }
            }

            if (t_tmp <= t && t < t_tmp + dt_grid && i < N) {
                double cursorVal_tmp = getWindowCoord(t, timeStart, timeEnd);
                char timeText[100];
                sprintf(timeText, "mpc t idx: %d", i);
                draw_list->AddText(ImVec2(p.x + cursorVal_tmp, p.y + rowOffset - 220), col_cursor, timeText);
            }
        }
    }

    cursorVal = getWindowCoord(t, timeStart, timeEnd);
    draw_list->AddLine(ImVec2(p.x + cursorVal, p.y), ImVec2(p.x + cursorVal, p.y + rowOffset), col_cursor);
    char timeText[100];
    sprintf(timeText, "t = %2.2lfs", t);
    draw_list->AddText(ImVec2(p.x + cursorVal - height / 2.0, p.y + rowOffset + height * 0.5), col_cursor, timeText);
    draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y - 5), 5, col_cursor);
    draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y + rowOffset + 5), 5, col_cursor);

    // and also show here the planning horizon...
    float horizonVal = getWindowCoord(timeStampForLastUpdate, timeStart, timeEnd);
    if (horizonVal <= labelWindowWidth + cpmWindowWidth) {
        draw_list->AddLine(ImVec2(p.x + horizonVal, p.y), ImVec2(p.x + horizonVal, p.y + rowOffset), col_horizon);
        char timeText[100];
        sprintf(timeText, "t = %2.2lfs", timeStampForLastUpdate);
        draw_list->AddText(ImVec2(p.x + horizonVal - height / 2.0, p.y + rowOffset + height * 0.5), col_horizon, timeText);
        draw_list->AddCircleFilled(ImVec2(p.x + horizonVal, p.y - 5), 5, col_horizon);
        draw_list->AddCircleFilled(ImVec2(p.x + horizonVal, p.y + rowOffset + 5), 5, col_horizon);
    }

    // finally draw the labels of the swing/stance phases...
    rowOffset = 0;
    if (drawLabels)
        for (const auto &lcs : cs.swings) {
            ContactPhaseInfo cpi = cs.getContactPhaseInformation(lcs.limbName, t);
            char text[100];
            if (cpi.isSwing()) {
                sprintf(text, "p:%2.0lf%%", cpi.getPercentageOfTimeElapsed() * 100);
                float labelPos = getWindowCoord(t + cpi.getTimeLeft() - cpi.getDuration(), timeStart, timeEnd);
                draw_list->AddText(ImVec2(p.x + labelPos + 15, p.y + rowOffset + height * 0.075), col_text_dark, text);
            } else {
                if (cpi.getTimeLeft() < 10) {
                    sprintf(text, "t-%2.2lfs", cpi.getTimeLeft());
                    float labelPos = getWindowCoord(t + cpi.getTimeLeft(), timeStart, timeEnd);
                    draw_list->AddText(ImVec2(p.x + labelPos + 15, p.y + rowOffset + height * 0.075), col_text_dark, text);
                } else {
                    sprintf(text, "inStance");
                    float labelPos = getWindowCoord(t, timeStart, timeEnd);
                    draw_list->AddText(ImVec2(p.x + labelPos + 15, p.y + rowOffset + height * 0.075), col_text_gray, text);
                }
            }
            rowOffset += height;
        }

    ImGui::End();
}
}  // namespace crl::loco