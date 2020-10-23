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

#include <atomic>
#include <mutex>
#include <memory>

class Servo // Interface
{
public:
    Servo(uint id);
    virtual ~Servo() {}

    virtual uint get_id() { return m_id; }

    virtual bool set_offset(int anglePoints) = 0;
    virtual bool set_max_temp(int temp) = 0;

    // Do servo move if needed and update temp if updateTemp is true
    virtual void do_cycle(bool lazyUpdate) = 0;

    virtual void set_goal_pos(int pos, int moveTime = 0);
    virtual int  get_goal_pos() { return m_goalPos; }

    virtual bool motor_off() = 0;

    // Return servo position and time when it was acquired
    virtual int get_current_pos(ros::Time* acquiredTime = nullptr);

    // Return servo temperature and time when it was acquired
    virtual int get_current_temp(ros::Time* acquiredTime = nullptr);

    virtual bool is_error();
    virtual void update_current_pos();
    virtual void update_temp();

protected:
    std::mutex m_mutexGoalPos;
    std::mutex m_mutexCurrPos;
    std::mutex m_mutexTemp;

    uint m_id;

    int m_goalPos;
    int m_goalMoveTime;
    std::atomic<bool> m_setPos;

    int m_currPos;
    ros::Time m_currPosTime;

    int m_currTemp;
    ros::Time m_currTempTime;

    std::atomic<bool> m_error;

    std::atomic<bool> m_updateCurrentPos;
    std::atomic<bool> m_updateTemp;
};

