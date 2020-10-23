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

#include <map>
#include <vector>
#include <memory>

#include "Leg.h"
#include "IK.h"
#include "Robot.h"

#define SET_DEF_PARAM(id, val) if (paramsInt.find(id) == paramsInt.end()) paramsInt[id] = val;

class BaseState;

class GCodeController // Interface
{
public:
    struct MovePoint {
        int legId;
        double p[3];
        int moveTime;
    };

    typedef std::map<uint8_t, std::shared_ptr<Leg>> RobotLegs;

    virtual void reset() = 0;

    virtual bool run(const std::string gcode, bool simulate = false) = 0;
    virtual bool do_move(bool simulate) = 0;

    virtual bool run_from_file(const std::string& file) = 0;

    virtual void say(const std::string& lang, const std::string& text) = 0;

    virtual bool add_point(int legId, IK::Vector pos, int moveTime) = 0;
    virtual bool look_at(IK::Vector vec) = 0;

    virtual int GetSpeedInPercent() = 0;
    virtual void SetSpeedInPercent(int speed) = 0;

    virtual RobotLegs  GetLegs() = 0;
    virtual BaseState* GetBaseState() = 0;
    virtual Robot*     GetRobot() = 0;

    virtual ros::Time GetMoveFinishTime() = 0;

    virtual IK::Vector get_rot() = 0;
};
