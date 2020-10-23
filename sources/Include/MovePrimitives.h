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
#pragma once

#include "MoveParams.h"

class GCodeController;

class MovePrimitives
{
public:
    MovePrimitives(GCodeController* ctrl): m_ctrl(ctrl) {}

    bool run_primitive(std::string cmd, std::map<int, int> params, bool simulate);

    bool set_pose(int posNum);

    std::string swing_leg(int legId, int height, int dist, double speedMultiply);
    std::string swing_leg_2(int legId1, int legId2, int height, int dist, double speedMultiply);

    bool walk     (int steps, bool forward);
    bool walk_2   (int steps, bool forward);
    bool side_walk(int steps, bool move_right);

    bool set_pose(int forward_offset, int side_offset, int heigh_offset);

    bool walk_3(int forward_step, int side_step, int rotate_step);
    bool walk_4(int forward_step, int side_step);

    bool rotate(int steps, bool clockwise);

    bool forward_jump(int steps, bool forward);
    bool Circle(int r);

    bool jump_2(int height, int forward, int side);

    bool run_gcode(std::string gcode);
    bool run_with_flush(std::string gcode);
    bool validate_gcode(std::string gcode);

    void add_leg_point     (Leg& leg, double x, double y, double z, ros::Time time, IK::Vector rot);
    void add_leg_wait_point(Leg& leg, double wait_time);
    void swing4_leg        (Leg& leg, double forward, double side, double height, double time, IK::Vector rot);

private:
    MoveParams m_params;
    GCodeController* m_ctrl;
};
