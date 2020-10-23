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

#include "Joint.h"
#include "Robot.h"
#include "Servos/Servo.h"

#include <memory>

class JointDouble : public Joint
{
public:
    JointDouble(std::shared_ptr<Servo> s1, std::shared_ptr<Servo> s2, std::shared_ptr<Robot> robot) :
        m_s1(s1), m_s2(s2), m_robot(robot)
    {
    }

    virtual bool move(double angle, int move_time, bool simulate) override
    {
        bool ret = true;

        int pos;
        if (m_robot->angle_to_servo_pos(m_s1->get_id(), angle, pos, simulate)) {
            if (!simulate) m_s1->set_goal_pos(pos, move_time);
        }
        else
            ret = false;

        if (m_robot->angle_to_servo_pos(m_s2->get_id(), angle, pos, simulate)) {
            if (!simulate) m_s2->set_goal_pos(pos, move_time);
        }
        else
            ret = false;

        return ret;
    }

private:
    std::shared_ptr<Servo> m_s1;
    std::shared_ptr<Servo> m_s2;
    std::shared_ptr<Robot> m_robot;
};

namespace Factory
{
    std::shared_ptr<Joint> CreateJointDouble(std::shared_ptr<Servo> s1, std::shared_ptr<Servo> s2, std::shared_ptr<Robot> robot)
    {
        return std::make_shared<JointDouble>(s1, s2, robot);
    }
}
