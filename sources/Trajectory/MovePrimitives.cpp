/////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 RoboLab19
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////
#include "IK.h"
#include "Utils.h"
#include "GCodeController.h"
#include "MovePrimitives.h"
#include "BaseState.h"

#include <string.h>
#include <math.h>
#include <iostream>

#define CHECK_MIN_PARAMS_COUNT(n)  if (params.size() < n) { printf("Not enough params (should be %d) in g-code cmd: %s\n", n, cmd.c_str()); return false; }

bool MovePrimitives::run_primitive(std::string cmd, std::map<int, int> params, bool simulate)
{
    if (cmd == "$POSE") {
        CHECK_MIN_PARAMS_COUNT(1)
        return set_pose(params[0]);
    }
    if (cmd == "$WALK") {
        CHECK_MIN_PARAMS_COUNT(2)
        return walk(params[0], params[1]);
    }
    else if (cmd == "$PARAM_ROTATE") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_rotate = params[0];
        return true;
    }
    else if (cmd == "$PARAM_OFFSET_X") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_offsetX = params[0];
        return true;
    }
    else if (cmd == "$PARAM_OFFSET_Y") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_offsetY = params[0];
        return true;
    }
    else if (cmd == "$PARAM_STEP") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_step = params[0];
        return true;
    }
    else if (cmd == "$PARAM_LEAN") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_lean = params[0];
        return true;
    }
    else if (cmd == "$PARAM_HEIGHT") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_height = params[0];
        return true;
    }
    else if (cmd == "$PARAM_START_STOP") {
        CHECK_MIN_PARAMS_COUNT(2)
        for (int i = 0; i < 2; ++i) m_params.m_startStop[i] = params[i];
        return true;
    }
    else if (cmd == "$PARAM_DIR") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_direction = params[0];
        return true;
    }
    else if (cmd == "$PARAM_STEP_SIZE") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_stepSize = params[0];
        return true;
    }
    else if (cmd == "$PARAM_PLANE") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_plane = params[0];
        return true;
    }
    else if (cmd == "$PARAM_POI") {
        CHECK_MIN_PARAMS_COUNT(3)
        for (int i = 0; i < 3; ++i) m_params.m_POI[i] = params[i];
        return true;
    }
    else if (cmd == "$PARAM_LOOK_AT") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_lookAt = params[0];
        return true;
    }
    else if (cmd == "$PARAM_RESET") {
        m_params.reset();
        return true;
    }
    else if (cmd == "$PARAM_WAIT") {
        CHECK_MIN_PARAMS_COUNT(1)
        m_params.m_wait = params[0];
        return true;
    }
    else if (cmd == "$WALK_2") {
        CHECK_MIN_PARAMS_COUNT(2)
        return walk_2(params[0], params[1]);
    }
    else if (cmd == "$WALK_3") {
        CHECK_MIN_PARAMS_COUNT(3)
        return walk_3(params[0], params[1], params[2]);
    }
    else if (cmd == "$WALK_4") {
        CHECK_MIN_PARAMS_COUNT(2)
        return walk_4(params[0], params[1]);
    }
    else if (cmd == "$SET_POSE") {
        CHECK_MIN_PARAMS_COUNT(3)
        return set_pose(params[0], params[1], params[2]);
    }
    else if (cmd == "$SIDE_WALK") {
        CHECK_MIN_PARAMS_COUNT(2)
        return side_walk(params[0], params[1]);
    }
    else if (cmd == "$ROTATE") {
        CHECK_MIN_PARAMS_COUNT(2)
        return rotate(params[0], params[1]);
    }
    else if (cmd == "$JUMP") {
        CHECK_MIN_PARAMS_COUNT(2)
        return forward_jump(params[0], params[1]);
    }
    else if (cmd == "$JUMP_2") {
        CHECK_MIN_PARAMS_COUNT(2)
        return jump_2(params[0], params[1], params[2]);
    }
    else if (cmd == "$CIRCLE") {
        CHECK_MIN_PARAMS_COUNT(1)
        return Circle(params[0]);
    }
    else if (cmd == "$BREAK") {
        printf("Break\n");
        return true;
    }

    printf("Unknown cmd: %s\n", cmd.c_str());
    return false;
}

bool MovePrimitives::set_pose(int posNum)
{
    switch (posNum)
    {
    case 0: // Down
        return run_with_flush("M 0 80 0\n WM\n");
    case 1: // Middle
        return run_with_flush("M 0 200 0\n WM\n");
    case 2: // Up
        return run_with_flush("M 0 330 0\n WM\n");
    case 3: // Sit
        return run_with_flush("L1 0 285 0\n L2 0 285 0\n L3 0 170 0\n L4 0 170 0\n WM\n");
    case 4: // Lean
        return run_with_flush("L1 0 90 50\n L2 0 90 50\n L3 0 250 50\n L4 0 250 50\n WM\n");
    case 5: // For run
        return run_with_flush("L1 20 200 -40\n L2 -20 200 -40\n L3 20 200 -40\n L4 -20 200 -40\n WM\n");
    }

    printf("Unknown pose: %d\n", posNum);
    return false;
}

std::string MovePrimitives::swing_leg(int legId, int height, int dist, double speedMultiply)
{
    std::string gcode;

    gcode += utils::printf("S %d\n", (int)(m_ctrl->GetSpeedInPercent() * speedMultiply));

    // Mask the swing leg from base rotate/offset adjustment
    int l[4];
    for (int i = 0; i < 4; ++i) l[i] = (i+1 == legId) ? 0 : 1;
    gcode += utils::printf("LEG_ADJ %d %d %d %d\n", l[0], l[1], l[2], l[3]);

    gcode += utils::printf("L%d+ 0 -%d  0\n WM\n", legId, height);
    gcode += utils::printf("L%d+ 0   0 %d\n WM\n", legId, dist/2);
    gcode += utils::printf("L%d+ 0   0 %d\n WM\n", legId, dist/2);
    gcode += utils::printf("L%d+ 0  %d  0\n WM\n", legId, height);

    gcode += utils::printf("LEG_ADJ 1 1 1 1\n");

    gcode += utils::printf("S %d\n", m_ctrl->GetSpeedInPercent());

    return gcode;
}

std::string MovePrimitives::swing_leg_2(int legId1, int legId2, int height, int dist, double speedMultiply)
{
    std::string gcode;

    gcode += utils::printf("S %d\n", (int)(m_ctrl->GetSpeedInPercent() * speedMultiply));

    // Mask the swing leg from base rotate/offset adjustment
    int l[4];
    for (int i = 0; i < 4; ++i) l[i] = (i+1 == legId1 || i+1 == legId2) ? 0 : 1;
    gcode += utils::printf("LEG_ADJ %d %d %d %d\n", l[0], l[1], l[2], l[3]);

    gcode += utils::printf("L%d+ 0 -%d  0\n"     , legId1, height);
    gcode += utils::printf("L%d+ 0 -%d  0\n WM\n", legId2, height);

    gcode += utils::printf("L%d+ 0   0 %d\n"     , legId1, dist/2);
    gcode += utils::printf("L%d+ 0   0 %d\n WM\n", legId2, dist/2);

    gcode += utils::printf("L%d+ 0   0 %d\n"     , legId1, dist/2);
    gcode += utils::printf("L%d+ 0   0 %d\n WM\n", legId2, dist/2);

    gcode += utils::printf("L%d+ 0  %d  0\n"     , legId1, height);
    gcode += utils::printf("L%d+ 0  %d  0\n WM\n", legId2, height);

    gcode += utils::printf("LEG_ADJ 1 1 1 1\n");

    gcode += utils::printf("S %d\n", m_ctrl->GetSpeedInPercent());

    return gcode;
}

bool MovePrimitives::walk(int steps, bool forward)
{
    std::string gcode;

    const int kStep = m_params.m_step;

    gcode += utils::printf("BASE_OFFSET %d 0 -%d\n", m_params.m_offsetX, 183 - m_params.m_height);
    gcode += utils::printf("BASE_ROT 0 0 %d\n", m_params.m_rotate);
    gcode += utils::printf("$POSE 5\n WM\n"); // Apply offset/rotation

    for (int i = 0; i < steps; ++i) {
        int forwardLean;
        if (forward)
            forwardLean = -kStep/2;
        else
            forwardLean = (i == 0) ? 0 : kStep/2;

        // Lean to left
        int leanLeft = (i == 0) ? m_params.m_lean/2 : m_params.m_lean;
        gcode += utils::printf("M+ -%d 0 %d\n WM\n", leanLeft, forwardLean);

        int dist = ((i == 0) ? kStep/2 : kStep) * (forward ? 1 : -1);

        gcode += swing_leg(3, 20, dist, 2);
        gcode += swing_leg(1, 20, dist, 2);

        if (forward)
            forwardLean = (i == steps-1) ? -kStep/4 : -kStep/2;
        else
            forwardLean = kStep/2;

        // Lean to right
        int leanRight = m_params.m_lean;
        gcode += utils::printf("M+ %d 0 %d\n WM\n", leanRight, forwardLean);

        dist = ((i == steps-1) ? kStep/2 : kStep) * (forward ? 1 : -1);

        double speedKoef = (i == steps-1) ? 2 : 3;
        gcode += swing_leg(4, 20, dist, speedKoef);
        gcode += swing_leg(2, 20, dist, speedKoef);
    }

    gcode += "S 100\n BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n";

    return run_with_flush(gcode);
}

bool MovePrimitives::walk_2(int steps, bool forward)
{
    std::string gcode;

    const int kStep = m_params.m_step / 2;

    gcode += utils::printf("BASE_OFFSET %d %d -%d\n", m_params.m_offsetX, m_params.m_offsetY, 200 - m_params.m_height);
    gcode += utils::printf("BASE_ROT 0 0 %d\n", m_params.m_rotate);
    gcode += utils::printf("M+ 0 0 0\n"); // Apply offset/rotation

    // To balance the first step
    gcode += swing_leg_2(1, 4, 20, 0, 2);

    for (int i = 0; i < steps; ++i) {
        int dist = forward ? kStep : -kStep;

        int forward = (i > 0) ? dist : dist/2;
        gcode += utils::printf("L2+ 0 0 %d\n", -forward);
        gcode += utils::printf("L3+ 0 0 %d\n", -forward);

        gcode += swing_leg_2(1, 4, 20, forward, 1.5);

        forward = dist;
        gcode += utils::printf("L1+ 0 0 %d\n", -forward);
        gcode += utils::printf("L4+ 0 0 %d\n", -forward);

        gcode += swing_leg_2(2, 3, 20, forward, 1.5);
    }

    gcode += "BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n$POSE 5\n";

    return run_with_flush(gcode);
}

bool MovePrimitives::side_walk(int steps, bool move_right)
{
    MoveParams savedParams = m_params;

    int side_step = m_params.m_step;

    m_params.m_step = 5;
    m_params.m_offsetY = move_right ? -side_step : side_step;

    bool ret = walk_2(steps, 1);
    m_params = savedParams;

    run_with_flush("BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n");

    return ret;
}

bool MovePrimitives::rotate(int steps, bool clockwise)
{
    std::string gcode;

    const int kStep = m_params.m_step / 7;

    gcode += utils::printf("BASE_OFFSET %d 0 -%d\n", m_params.m_offsetX, 200 - m_params.m_height);
    gcode += utils::printf("BASE_ROT 0 0 %d\n", clockwise ? kStep : -kStep);

    int height = 30;

    for (int i = 0; i < steps; ++i) {
        gcode += utils::printf("LEG_ADJ 0 1 1 0\n");
        gcode += utils::printf("M+ 0 0 0\n");

        gcode += utils::printf("L1+ 0 -%d  0\n",      height);
        gcode += utils::printf("L4+ 0 -%d  0\n WM\n", height);
        gcode += utils::printf("L1+ 0  %d  0\n"     , height);
        gcode += utils::printf("L4+ 0  %d  0\n WM\n", height);

        gcode += utils::printf("LEG_ADJ 1 0 0 1\n");
        gcode += utils::printf("M+ 0 0 0\n");

        gcode += utils::printf("L2+ 0 -%d  0\n",      height);
        gcode += utils::printf("L3+ 0 -%d  0\n WM\n", height);
        gcode += utils::printf("L2+ 0  %d  0\n"     , height);
        gcode += utils::printf("L3+ 0  %d  0\n WM\n", height);
    }

    gcode += utils::printf("LEG_ADJ 1 1 1 1\n");
    gcode += "BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n";

    return run_with_flush(gcode);
}

bool MovePrimitives::walk_3(int forward_step, int side_step, int rotate_step)
{
    std::string gcode;

    gcode += utils::printf("BASE_OFFSET %d %d 0\n", forward_step, -side_step);
    //gcode += utils::printf("BASE_ROT %d %d %d\n", (int)IK::toDeg(m_ctrl->get_rot().x), (int)IK::toDeg(m_ctrl->get_rot().y), rotate_step);
    //gcode += utils::printf("BASE_ROT 0 0 %d\n", rotate_step);

    int height = 20; //23;

    // Move base (leg 1 and 4) according to BASE_OFFSET without legs 2 and 3
    gcode += utils::printf("LEG_ADJ 0 1 1 0\n");
    gcode += utils::printf("M+ 0 0 0\n");

    // Move leg 1 and 4 up and down without BASE_OFFSET
    gcode += utils::printf("L1+ 0 -%d  0\n",       height);
    gcode += utils::printf("L4+ 0 -%d  0\n WM3\n", height);
    gcode += utils::printf("L1+ 0  %d  0\n"      , height);
    gcode += utils::printf("L4+ 0  %d  0\n WM3\n", height);

    // Move base (leg 2 and 3) according to BASE_OFFSET without legs 1 and 4
    gcode += utils::printf("LEG_ADJ 1 0 0 1\n");
    gcode += utils::printf("M+ 0 0 0\n");

    // Move leg 2 and 3 up and down without BASE_OFFSET
    gcode += utils::printf("L2+ 0 -%d  0\n",       height);
    gcode += utils::printf("L3+ 0 -%d  0\n WM3\n", height);
    gcode += utils::printf("L2+ 0  %d  0\n"     ,  height);
    gcode += utils::printf("L3+ 0  %d  0\n WM3\n", height);

    // Prepear to adjust all legs to new pos (if needed)
    // NB! It will be skipped if walk_3 will be called again
    gcode += utils::printf("LEG_ADJ 1 1 1 1\n");
    gcode += "BASE_OFFSET 0 0 0\n";

    return run_with_flush(gcode);
}

void MovePrimitives::add_leg_point(Leg& leg, double x, double y, double z, ros::Time time, IK::Vector rot)
{
    m_ctrl->GetBaseState()->RotateInBase  (leg.getId(), x, y, z, rot);
    m_ctrl->GetBaseState()->AdjustByOffset(leg.getId(), x, y, z);

    leg.add_point(x, y, z, time);
}

void MovePrimitives::add_leg_wait_point(Leg& leg, double wait_time)
{
    auto p = leg.get_final_point();
    leg.add_point(p.x, p.y, p.z, p.time + ros::Duration(wait_time));
}

double g_height = 0.150;

void MovePrimitives::swing4_leg(Leg& leg, double forward, double side, double height, double time, IK::Vector rot)
{
    //std::cout << "swing4_leg\n";
    auto p = leg.get_final_point();

    double k1 = 1.0 / 4.0;
    double k2 = 2.0 / 4.0;
    double k3 = 3.0 / 4.0;

    p.z = forward*k1;
    p.x = side*k1;
    p.y = g_height - height*2/3.0;
    auto t = p.time + ros::Duration(time*k1);
    add_leg_point(leg, p.x, p.y, p.z, t, rot);

    p.z = forward*k2;
    p.x = side*k2;
    p.y = g_height - height;
    t   = p.time + ros::Duration(time*k2);
    add_leg_point(leg, p.x, p.y, p.z, t, rot);

    p.z = forward*k3;
    p.x = side*k3;
    p.y = g_height - height*2/3;
    t   = p.time + ros::Duration(time*k3);
    add_leg_point(leg, p.x, p.y, p.z, t, rot);

    p.z = forward;
    p.x = side;
    p.y = g_height;
    t   = p.time + ros::Duration(time);
    add_leg_point(leg, p.x, p.y, p.z, t, rot);
}

bool MovePrimitives::walk_4(int forward_step, int side_step)
{
    GCodeController::RobotLegs legs = m_ctrl->GetLegs();

    if (abs(forward_step) < 20) forward_step = 20 * sign(forward_step);
    if (abs(side_step)    < 10) side_step    = 10 * sign(side_step);

    //std::cout << "forward_step: " << forward_step << " side_step: " << side_step << "\n";

    int rotate_step = IK::toDeg(m_ctrl->GetBaseState()->m_rot.z);

    int max_step = std::max(abs(forward_step), abs(side_step));
    max_step = std::max(max_step, abs(2*rotate_step));

    const double kStepK = 0.09 / 50;
    auto time = 0.10 + max_step * kStepK;
    double height1 = 0.01 + (0.01 * (max_step / 50.0));
    double height2 = -0.005;

    if (m_ctrl->GetRobot()->GetVer() == 2) {
        height2 = 0;
    }

    //std::cout << "height1: " << height1 << " max_step: " << max_step << "\n";

    auto rot1 = m_ctrl->GetBaseState()->m_rot;
    auto rot2 = m_ctrl->GetBaseState()->m_rot;
    rot1.z = 0;

    double forward = forward_step / 1000.0;
    double side    = side_step    / 1000.0;

    swing4_leg(*legs[1], forward, -side, height1, time, rot1);
    swing4_leg(*legs[4], forward, -side, height1, time, rot1);

    swing4_leg(*legs[2], -forward, side, height2, time, rot2);
    swing4_leg(*legs[3], -forward, side, height2, time, rot2);

    swing4_leg(*legs[2], forward, -side, height1, time, rot1);
    swing4_leg(*legs[3], forward, -side, height1, time, rot1);

    swing4_leg(*legs[1], -forward, side, height2, time, rot2);
    swing4_leg(*legs[4], -forward, side, height2, time, rot2);

    return true;
}

bool MovePrimitives::jump_2(int height, int forward, int side)
{
    double time = 0.1;

    IK::Vector rot;

    double h = height  / 1000.0;
    double f = forward / 1000.0;
    double s = side    / 1000.0;

    //std::cout << "height: " << height << "\n";

    for (auto& l : m_ctrl->GetLegs()) {
        auto& leg = *l.second;

        auto p = leg.get_final_point();

        double k1 = 1.0 / 2.0;

        p.z = f;
        p.x = s;
        p.y = g_height + h;
        auto t = p.time + ros::Duration(time*k1);
        add_leg_point(leg, p.x, p.y, p.z, t, rot);

        p.z = 0;
        p.x = 0;
        p.y = g_height;
        t = p.time + ros::Duration(time);
        add_leg_point(leg, p.x, p.y, p.z, t, rot);

        add_leg_wait_point(leg, 0.3);
    }
}

bool MovePrimitives::set_pose(int forward_offset, int side_offset, int heigh_offset)
{
    int x =  0; //25;
    int y = (m_ctrl->GetRobot()->GetVer() == 2) ? 170 : 200;
    int z = 0; //-40;

    g_height = (y + heigh_offset) / 1000.0;

    std::string gcode;
    gcode += utils::printf("L1 %d %d %d\n",  x+side_offset, y+heigh_offset, z-forward_offset);
    gcode += utils::printf("L2 %d %d %d\n", -x+side_offset, y+heigh_offset, z-forward_offset);
    gcode += utils::printf("L3 %d %d %d\n",  x+side_offset, y+heigh_offset, z-forward_offset);
    gcode += utils::printf("L4 %d %d %d\n", -x+side_offset, y+heigh_offset, z-forward_offset);
    gcode += "WM\n";

    return run_with_flush(gcode);
}

bool MovePrimitives::forward_jump(int steps, bool forward)
{
    std::string gcode;

    const int kStep = 50;

    if (forward)
        gcode += "S 250\n BASE_OFFSET 0 0 0\n";
    else
        gcode += "S 250\n BASE_OFFSET 70 0 0\n";

    gcode += "$POSE 1\n";

    for (int i = 0; i < steps; ++i) {
        if (forward) {
            gcode += utils::printf("L1 0 220 0\n");
            gcode += utils::printf("L2 0 220 0\n WM\n");

            gcode += utils::printf("L3 0 210 %d\n", -kStep);
            gcode += utils::printf("L4 0 210 %d\n WM\n", -kStep);
        }
        else {
            gcode += utils::printf("L3 0 220 0\n");
            gcode += utils::printf("L4 0 220 0\n WM\n");

            gcode += utils::printf("L1 0 210 %d\n", kStep);
            gcode += utils::printf("L2 0 210 %d\n WM\n", kStep);
        }

        gcode += utils::printf("L1 0 200 0\n");
        gcode += utils::printf("L2 0 200 0\n");
        gcode += utils::printf("L3 0 200 0\n");
        gcode += utils::printf("L4 0 200 0\n WM2\n");
    }

    gcode += "S 100\n BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n WM\n";

    return run_with_flush(gcode);
}

bool MovePrimitives::Circle(int r)
{
    std::string gcode;

    double startAngle = IK::toRad(m_params.m_startStop[0]);
    double stopAngle  = IK::toRad(m_params.m_startStop[1]);

    if (m_params.m_direction) {
        if (startAngle >= stopAngle) startAngle -= 2*PI;
    }
    else {
        if (startAngle <= stopAngle) startAngle += 2*PI;
    }

    int x = m_params.m_POI[0];
    int y = m_params.m_POI[1];
    int z = m_params.m_POI[2];

    int prevX = 0;
    int prevY = 0;
    int prevZ = 0;

    double steps_per_circle = (2 * PI * r) / m_params.m_stepSize;
    double step = m_params.m_direction ? 2*PI/steps_per_circle : -2*PI/steps_per_circle;
    for (double t = startAngle; ; t += step) {
        if (m_params.m_direction) {
            if (t > stopAngle) break;
        }
        else {
            if (t < stopAngle) break;
        }

        double a = (PI/2 - t) + PI;
        int xx = 0;
        int yy = 0;
        int zz = 0;

        switch (m_params.m_plane) {
        case 0: // XY plane
            xx = r * sin(a) + x;
            yy = r * cos(a) + y;
            zz = z;
            break;
        case 1: // XZ plane
            xx = r * sin(a) + x;
            yy = y;
            zz = r * cos(a) + z;
            break;
        case 2: // YZ plane
            xx = x;
            yy = r * cos(a) + y;
            zz = r * sin(a) + z;
            break;
        default:
            printf("Unknown plane: %d\n", m_params.m_plane);
            return false;
        }

        // Add wait for the first point
        if (m_params.m_wait != 0) {
            gcode += utils::printf("W %d\n", m_params.m_wait);
            m_params.m_wait = 0;
        }

        if (m_params.m_lookAt) {
            gcode += utils::printf("LOOK_AT %d %d %d\n M+ 0 0 0\n WM\n", xx, yy, zz);
        }
        else
            gcode += utils::printf("BASE_OFFSET %d %d %d\n M+ 0 0 0\n WM\n", xx, yy, zz);

        prevX = xx;
        prevY = yy;
        prevZ = zz;
    }

    gcode += utils::printf("BASE_OFFSET 0 0 0\n BASE_ROT 0 0 0\n");

    return run_with_flush(gcode);
}

bool MovePrimitives::run_gcode(std::string gcode)
{
    // Save rotation matrix and speedFactor
    auto speed = m_ctrl->GetSpeedInPercent();
    bool ret   = m_ctrl->run(gcode, false);

    // Restore rotation matrix and speedFactor
    m_ctrl->SetSpeedInPercent(speed);

    return true;
}

bool MovePrimitives::run_with_flush(std::string gcode)
{
    return run_gcode(gcode + "FLUSH \n");
}

bool MovePrimitives::validate_gcode(std::string gcode)
{
    return m_ctrl->run(gcode, true);
}
