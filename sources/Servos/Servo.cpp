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
#include "Servos/Servo.h"

Servo::Servo(uint id) :
    m_id(id), m_goalPos(0), m_goalMoveTime(0),
    m_error(false), m_setPos(false),
    m_updateCurrentPos(false), m_updateTemp(false)
{
}

void Servo::set_goal_pos(int pos, int moveTime)
{
    std::lock_guard<std::mutex> guard(m_mutexGoalPos);

    m_goalPos      = pos;
    m_goalMoveTime = moveTime;

    m_setPos.store(true);
}

int Servo::get_current_pos(ros::Time* acquiredTime)
{
    std::lock_guard<std::mutex> guard(m_mutexCurrPos);

    if (acquiredTime) *acquiredTime = m_currPosTime;
    return m_currPos;
}

int Servo::get_current_temp(ros::Time* acquiredTime)
{
    std::lock_guard<std::mutex> guard(m_mutexTemp);

    if (acquiredTime) *acquiredTime = m_currTempTime;
    return m_currTemp;
}

bool Servo::is_error()
{
    return m_error.load();
}

void Servo::update_current_pos()
{
    m_updateCurrentPos.store(true);
}

void Servo::update_temp()
{
    m_updateTemp.store(true);
}
