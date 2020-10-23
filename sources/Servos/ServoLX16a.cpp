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
#include "Servos/LX16a.h"
#include "Servos/Servo.h"

const int kWarnTemp = 50;

class ServoLX16a : public Servo
{
public:
    ServoLX16a(int servoId, std::shared_ptr<LX16a> lx16a) :
        Servo(servoId), m_lx16a(lx16a)
    {
    }

    virtual bool set_offset(int anglePoints) override
    {
        if (!m_lx16a->set_offset(m_id, anglePoints)) {
            printf("Failed to set offset %d for servo %d!\n", anglePoints, m_id);
            m_error.store(true);
            return false;
        }
        return true;
    }

    virtual bool set_max_temp(int temp) override
    {
        if (!m_lx16a->set_max_temp(m_id, temp)) {
            printf("Failed to set max_temp %d for servo %d!\n", temp, m_id);
            m_error.store(true);
            return false;
        }
        return true;
    }

    virtual bool motor_off() override
    {
        return m_lx16a->motor_off(m_id);
    }

    virtual void do_cycle(bool lazyUpdate) override
    {
        if (true == m_setPos.load()) {
            int pos, moveTime;
            {
                std::lock_guard<std::mutex> guard(m_mutexGoalPos);

                pos      = m_goalPos;
                moveTime = m_goalMoveTime;
            }

            if (!m_lx16a->move(m_id, pos, moveTime)) {
                m_error.store(true);
                return;
            }
            m_setPos.store(false);
        }

        if (true == m_updateCurrentPos.load()) {
            int16_t pos;
            if (!m_lx16a->get_pos(m_id, pos)) {
                m_error.store(true);
                return;
            }

            {
                std::lock_guard<std::mutex> guard(m_mutexCurrPos);

                m_currPos     = pos;
                m_currPosTime = ros::Time::now();

                m_updateCurrentPos.store(false);
            }
        }

        if (lazyUpdate || true == m_updateTemp.load()) {
            uint8_t temp;
            if (!m_lx16a->get_temp(m_id, temp))
            {
                m_error.store(true);
                return;
            }

            {
                std::lock_guard<std::mutex> guard(m_mutexTemp);

                m_currTemp = temp;
                m_currTempTime = ros::Time::now();

                if (temp > kWarnTemp) printf("High temp (%d) for servo %d\n", temp, m_id);

                m_updateTemp.store(false);
            }
        }

        m_error.store(false);
    }

private:
    std::shared_ptr<LX16a> m_lx16a;
};

namespace Factory
{
    std::shared_ptr<Servo> CreateLX16a(uint servoId, std::shared_ptr<LX16a> lx16a, int offset)
    {
        auto servo = std::make_shared<ServoLX16a>(servoId, lx16a);
        servo->set_offset(offset);

        return servo;
    }
}
