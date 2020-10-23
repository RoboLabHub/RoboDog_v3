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

#include "Robot.h"
#include "Leg.h"
#include "Servos/LX16a.h"
#include "Servos/Servo.h"
#include "Servos/ServoController.h"
#include "Joint.h"
#include "GCodeController.h"

#include <map>
#include <memory>

namespace Factory
{
    std::shared_ptr<Leg> CreateLeg(int legId, std::map<uint, std::shared_ptr<Joint>> joints, bool inverseX);
    std::shared_ptr<Servo> CreateLX16a(uint servoId, std::shared_ptr<LX16a> lx16a, int offset);

    std::shared_ptr<GCodeController> CreateGCodeController(GCodeController::RobotLegs legs, std::shared_ptr<Robot> robot);

    std::shared_ptr<Joint> CreateJointSingle(std::shared_ptr<Servo> servo, std::shared_ptr<Robot> robot);
    std::shared_ptr<Joint> CreateJointDouble(std::shared_ptr<Servo> s1, std::shared_ptr<Servo> s2, std::shared_ptr<Robot> robot);

    bool CreateRobotV2(std::shared_ptr<Robot>& robot);
    bool CreateRobotV3(std::shared_ptr<Robot>& robot);
};
