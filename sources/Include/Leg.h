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

#include "ros/ros.h"
#include "IK.h"

class Leg // Interface
{
public:
    struct MOVE_POINT {
        double x, y, z;
        ros::Time time;
    };

    virtual int getId() = 0;

    virtual void set_leg_location(double x, double y, double z) = 0; // In base frame (in m)
    virtual IK::Vector get_leg_location() = 0;

    virtual void set_geometry(double link1, double link2, double link3) = 0;

    virtual bool move(double angles[3], int move_time, bool simulate = false) = 0;

    virtual bool move(double x, double y, double z, int move_time = 1000, bool simulate = false) = 0;

    virtual bool init_with_pos(double x, double y, double z) = 0;

    virtual bool add_point(double x, double y, double z, const ros::Time& time) = 0;
    virtual MOVE_POINT get_final_point() = 0;

    virtual void cycle() = 0;

    virtual void tune_pos_1(bool useIK = false) = 0;
    virtual void tune_pos_2(bool useIK = false) = 0;
};
