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
#include "Servos/ServoController.h"
#include "Utils.h"

#include <unistd.h>

const double defaultLazyCheckPeriodInSec = 5.0;

ServoController::ServoController() : m_exit(false)
{
}

ServoController::~ServoController()
{
    Stop();
}

void ServoController::Stop()
{
    // Signal to all threads to exit
    m_exit.store(true);

    for (auto& group : m_groups) {
        group->thread->join();
    }

    ServoOff();
}


void ServoController::ServoOff()
{
    for (auto& group : m_groups) {
        for (auto& s : group->servos) {
            s->motor_off();
        }
    }
}

void ServoController::Diagnostics()
{
    for (auto& group : m_groups) {
        for (auto& s : group->servos) {
            s->update_temp();
            s->update_current_pos();
        }
    }

    utils::delay_ms(500);

    for (auto& group : m_groups) {
        for (auto& s : group->servos) {
            int id   = s->get_id();
            int goal = s->get_goal_pos();
            int pos  = s->get_current_pos();
            int temp = s->get_current_temp();

            auto str = utils::printf("id: %2d  goal: %3d  pos: %3d (%4d)  temp: %2dC\n", id, goal, pos, pos-goal, temp);
            std::cout << str.c_str();
        }
    }
}

int ServoController::GetMaxServoTemp()
{
    double maxTemp = 0;
    for (auto& group : m_groups) {
        for (auto& s : group->servos) {
            int temp = s->get_current_temp();
            if (temp > maxTemp) maxTemp = temp;
        }
    }
    return maxTemp;
}

void ServoController::ThreadBody(std::shared_ptr<SERVO_GROUP> group)
{
    int idToCheck = 0;
    auto timeToCheck = ros::Time::now();

    while (false == m_exit.load()) {
        //auto cycleTime = ros::Time::now();

        int id = 0;
        for (auto& servo : group->servos) {
            bool lazyUpdate = false;

            if (id == idToCheck && ros::Time::now() >= timeToCheck) {
                lazyUpdate = true;

                //printf("Lazy check for servo index %d\n", idToCheck);

                timeToCheck = ros::Time::now() + ros::Duration(defaultLazyCheckPeriodInSec / group->servos.size());
                if (++idToCheck >= group->servos.size()) idToCheck = 0;
            }

            servo->do_cycle(lazyUpdate);
            id++;
        }

        //auto diff = ros::Time::now() - cycleTime;
        //printf("%.6f\n", diff.toSec());

        // Sleep to avoid 100% CPU usage
        usleep(1); // 0.001 ms
    }
}

void ServoController::AddServoGroup(std::vector<std::shared_ptr<Servo>> servos)
{
    auto group = std::make_shared<SERVO_GROUP>();

    // Each servo group served by its own thread
    group->servos = servos;
    group->thread = new boost::thread(boost::bind(&ServoController::ThreadBody, this, group));

    m_groups.push_back(group);
}
