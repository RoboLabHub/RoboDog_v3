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
#include <fstream>
#include <sstream> //std::stringstream

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "SoundService.h"

#include "Utils.h"
#include "GCodeController.h"
#include "Servos/ServoController.h"

#include "BaseState.h"
#include "MovePrimitives.h"

using namespace boost;

class GCodeControllerImpl : public GCodeController
{
public:
    GCodeControllerImpl(RobotLegs legs, std::shared_ptr<Robot> robot): m_robot(robot), m_legs(legs), m_exit(false)
    {
        m_BaseState  = std::make_shared<BaseState>(this);
        m_primitives = std::make_shared<MovePrimitives>(this);

        reset();

        m_moveThread  = new boost::thread(boost::bind(&GCodeControllerImpl::ThreadBody, this));
    }

    ~GCodeControllerImpl()
    {
        m_exit.store(true);
        m_moveThread->join();
    }

    void ThreadBody()
    {
        while (false == m_exit.load()) {
            for (auto& l : m_legs) {
                l.second->cycle();
            }
            usleep(100); // 0.1 ms
        }

        std::cout << "GCode ThreadBody exit\n";
    }

    virtual void reset() override
    {
        m_speedInPercent = 100;

        m_movePoints.clear();
        m_BaseState->Init();
    }

    #define CHECK_MIN_PARAMS_COUNT(n)  if (paramsInt.size() < n) { printf("Not enough params (should be %d) in g-code: %s\n", n, line.c_str()); return false; }

    IK::Vector paramsToVector(std::map<int, int> paramsInt)
    {
        IK::Vector v;
        v.x = paramsInt[0] / 1000.0;
        v.y = paramsInt[1] / 1000.0;
        v.z = paramsInt[2] / 1000.0;

        return v;
    }

    double getSpeed(int speedInPercent)
    {
        return kMaxSpeed * speedInPercent / 100.0;
    }

    virtual IK::Vector get_rot() override
    {
        return m_BaseState->m_rot;
    }

    virtual bool run(const std::string gcode, bool simulate) override
    {
        tokenizer<char_separator<char> > lines(gcode, char_separator<char>("\n"));
        for (auto line: lines) {
            tokenizer<char_separator<char> > tokens(line, char_separator<char>(" "));

            auto it = tokens.begin();
            if (it.at_end()) continue;

            std::string cmd = utils::get_trim(it->c_str());
            if (cmd.substr(0, 1) == "#" || cmd.substr(0, 2) == "//") continue; // Skip commented line

            it++;

            // Tokens to map
            std::map<int, std::string> paramsStr;
            std::map<int, int> paramsInt;
            for (int n = 0; !it.at_end(); it++, n++) {
                auto str = utils::get_trim(*it);
                paramsStr[n] = str;
                paramsInt[n] = atoi(str.c_str());
            }

            if (cmd == "S") {
                CHECK_MIN_PARAMS_COUNT(1)
                m_speedInPercent = paramsInt[0];     // speed in %
            }
            else if (cmd.length() == 2 && cmd[0] == 'L') { // Set leg position
                CHECK_MIN_PARAMS_COUNT(3)

                int legId = cmd[1] - '0';
                m_BaseState->MoveLeg(legId, paramsToVector(paramsInt), getSpeed(m_speedInPercent));
            }
            else if (cmd.length() == 3 && cmd[0] == 'L' && cmd[2] == '+') {
                CHECK_MIN_PARAMS_COUNT(3)

                int legId = cmd[1] - '0';
                IK::Vector pos = m_BaseState->GetPosByVector(legId, paramsToVector(paramsInt));
                m_BaseState->MoveLeg(legId, pos, getSpeed(m_speedInPercent));
            }
            else if (cmd == "M") { // Move base (absolute)
                CHECK_MIN_PARAMS_COUNT(3)

                IK::Vector pos = paramsToVector(paramsInt);
                if (!m_BaseState->SetBase(pos, getSpeed(m_speedInPercent))) return false;
            }
            else if (cmd == "M+") { // Move base (relative)
                CHECK_MIN_PARAMS_COUNT(3)

                IK::Vector v = paramsToVector(paramsInt);
                if (!m_BaseState->MoveBase(v, getSpeed(m_speedInPercent))) return false;
            }
            else if (cmd == "BASE_ROT") {  // Set base rotation
                CHECK_MIN_PARAMS_COUNT(3)

                m_BaseState->m_rot.x = IK::toRad(paramsInt[0]);
                m_BaseState->m_rot.y = IK::toRad(paramsInt[1]);
                m_BaseState->m_rot.z = IK::toRad(paramsInt[2]);
            }
            else if (cmd == "BASE_OFFSET") { // Set base offset
                CHECK_MIN_PARAMS_COUNT(3)

                m_BaseState->m_baseOffset = paramsToVector(paramsInt);
            }
            else if (cmd == "LOOK_AT") {
                CHECK_MIN_PARAMS_COUNT(3)

                IK::Vector vec = paramsToVector(paramsInt);
                if (!look_at(vec)) return false;
            }
            else if (cmd == "LEG_ADJ") { // Leg to adjust
                CHECK_MIN_PARAMS_COUNT(4)

                for (int i = 0; i < 4; ++i) m_BaseState->m_legToAdjust[i] = paramsInt[i];
            }
            else if (cmd == "SERVO_OFF") { // Switch all servo motors off
                m_robot->GetServoCtrl()->ServoOff();
            }
            else if (cmd == "SAY") {
                CHECK_MIN_PARAMS_COUNT(2)
                std::string text;
                for (size_t i = 1; i < paramsStr.size(); ++i) text += paramsStr[i] + " "; // Joint params into single string
                say(paramsStr[0], text);
            }
            else if (cmd == "PLAY_SOUND") {
                CHECK_MIN_PARAMS_COUNT(1)
                play_sound(paramsStr[0]);
            }
            else if (cmd == "FLUSH") {    // Do pending movements (to remove dependency between sequences during spline moves)
                do_move(simulate);
            }
            else {
                if (!do_move(simulate)) return false;

                if (cmd == "W") {   // Wait (in ms)
                    CHECK_MIN_PARAMS_COUNT(1)
                    utils::delay_ms(paramsInt[0]);
                }
                else if (cmd == "WM") {   // Wait for move completion
                    m_BaseState->WaitForMove();
                }
                else if (cmd == "WM2") {   // Wait for move completion (1.5 times longer)
                    m_BaseState->WaitForMove(1.5);
                }
                else if (cmd == "WM3") {   // Wait for move completion (3 times longer)
                    m_BaseState->WaitForMove(3);
                }
                else if (cmd[0] == '$') {
                    if (!m_primitives->run_primitive(cmd, paramsInt, simulate)) return false;
                }
                else {
                    printf("Unknown command: %s\n", cmd.c_str());
                    return false;
                }
            }
        }

        return do_move(simulate);
    }

    virtual bool add_point(int legId, IK::Vector pos, int moveTime) override
    {
        // Validate point first
        if (!m_legs[legId]->move(pos.x, pos.y, pos.z, moveTime, true)) {
            printf("Cannot move leg %d to [%.3f %.3f %.3f]!\n", legId, pos.x, pos.y, pos.z);
            return false;
        }

        MovePoint movePoint;
        movePoint.legId    = legId;
        movePoint.moveTime = moveTime;

        movePoint.p[0] = pos.x;
        movePoint.p[1] = pos.y;
        movePoint.p[2] = pos.z;

        m_movePoints.emplace_back(movePoint);
        return true;
    }

    virtual bool look_at(IK::Vector vec) override
    {
        // double mag = sqrt(dx*dx + dy*dy + dz*dz);
        // dx /= mag;
        // dy /= mag;
        // dz /= mag;

        double pitch;
        double yaw;
        IK::vectorToEuler(vec.x, vec.y, vec.z, pitch, yaw);

        m_BaseState->m_rot.x = 0;
        m_BaseState->m_rot.y = pitch;
        m_BaseState->m_rot.z = yaw;

        return true;
    }

    virtual bool do_move(bool simulate) override
    {
        for (auto p: m_movePoints) {
            //if (!m_legs[p.legId]->move(p.p[0], p.p[1], p.p[2], p.moveTime, simulate)) return false;

            auto last = m_legs[p.legId]->get_final_point();

            ros::Time time = last.time + ros::Duration(p.moveTime / 1000.0);
            if (!m_legs[p.legId]->add_point(p.p[0], p.p[1], p.p[2], time)) return false;
        }

        if (!simulate)
            m_movePoints.clear();

        return true;
    }

    virtual ros::Time GetMoveFinishTime() override
    {
        ros::Time time = ros::Time::now();
        for (auto& l : m_legs) {
            auto p = l.second->get_final_point();
            if (p.time > time) time = p.time;
        }

        return time;
    }

    virtual bool run_from_file(const std::string& path) override
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            printf("GCodeController::run_from_file(): Failed to open file: %s\n", path.c_str());
            return false;
        }

        std::stringstream strStream;
        strStream << file.rdbuf();

        return run(strStream.str(), false);
    }

    virtual void say(const std::string& lang, const std::string& text) override
    {
        robodog::SayService::Request req;
        robodog::SayService::Response res;
        req.lang = lang;
        req.text = text;
        ::say(req, res);
    /*
        ros::NodeHandle n;

        ros::ServiceClient client = n.serviceClient<robodog::SayService>("say");
        robodog::SayService srv;
        srv.request.text = text;
        if (client.call(srv)) {
            //ROS_INFO("Say OK");
        }
        else
            ROS_INFO("Say Failed");
    */
    }

    virtual int  GetSpeedInPercent() override { return m_speedInPercent; }
    virtual void SetSpeedInPercent(int speed) override { m_speedInPercent = speed; }

    virtual RobotLegs  GetLegs()      override { return m_legs; }
    virtual BaseState* GetBaseState() override { return m_BaseState.get(); }
    virtual Robot*     GetRobot()     override { return m_robot.get(); }

private:
    int m_speedInPercent;

    std::atomic<bool> m_exit;
    boost::thread*    m_moveThread;

    std::shared_ptr<BaseState>      m_BaseState;
    std::shared_ptr<MovePrimitives> m_primitives;

    RobotLegs m_legs;
    std::shared_ptr<Robot> m_robot;

    std::vector<MovePoint> m_movePoints;
};

namespace Factory
{
    std::shared_ptr<GCodeController> CreateGCodeController(GCodeController::RobotLegs legs, std::shared_ptr<Robot> robot)
    {
        return std::make_shared<GCodeControllerImpl>(legs, robot);
    }
}
