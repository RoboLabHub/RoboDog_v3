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
#include <iostream>

#include "Utils.h"
#include "RemoteController.h"
#include "Servos/ServoController.h"

using namespace std;

const double kTelemetrySendPeriodInSec = 2.0;

void CommandMenu::send_telemetry(bool force)
{
    static auto timeToSend = ros::Time::now();
    static auto timeToBeep = ros::Time::now();

    if (force || ros::Time::now() >= timeToSend) {
        uint8_t battVol = 111;
        uint8_t maxTemp = m_rc->m_robot->GetServoCtrl()->GetMaxServoTemp();

        //cout << "send telemetry: " <<  (int)maxTemp << " " << (int)battVol << "\n";

        if (maxTemp > 60) {
            m_rc->send_beep(2);
        }
        else if (ros::Time::now() >= timeToBeep) {
            m_rc->send_beep(1);
            timeToBeep = ros::Time::now() + ros::Duration(10.0);
        }

        // send_cmd(kCmdClear);
        m_rc->send_msg(5, 0, utils::printf("%2dC %2d.%dV", maxTemp, battVol/10, battVol%10).c_str());

        timeToSend = ros::Time::now() + ros::Duration(kTelemetrySendPeriodInSec);
    }
}

void CommandMenu::process_command()
{
    //m_rc->send_msg(3, 0, ctrl->m_flip ? "FLIP" : "    ");

    m_rc->send_msg(5, 1, "          ");
}