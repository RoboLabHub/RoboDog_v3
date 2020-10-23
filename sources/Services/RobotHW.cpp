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
#include "Utils.h"
#include "MathHelpers.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf2/LinearMath/Transform.h>
#include "fiducial_msgs/FiducialTransformArray.h"

#include "robodog/SayService.h"
#include "robodog/GcodeService.h"

#include "RemoteController.h"

#include "Factory.h"

GCodeController* g_ctrl = nullptr;

bool say_callback(robodog::SayService::Request& req, robodog::SayService::Response& res);

bool gcode_callback(robodog::GcodeService::Request& req, robodog::GcodeService::Response& res)
{
    //ROS_INFO("Received g-code: [%s]", req.gcode.c_str());
    res.result = g_ctrl->run(req.gcode);
    return true;
}

int main(int argc, char **argv)
{
    ros::Time::init();

    std::shared_ptr<Robot> robot;
    //if (!Factory::CreateRobotV2(robot)) return -1;
    if (!Factory::CreateRobotV3(robot)) return -1;

    g_ctrl = robot->GetGcodeCtrl();

    // g_ctrl->GetLegs()[1]->tune_pos_2(false);
    // g_ctrl->GetLegs()[2]->tune_pos_2(false);
    // g_ctrl->GetLegs()[3]->tune_pos_2(false);
    // g_ctrl->GetLegs()[4]->tune_pos_2(false);
    // utils::delay_ms(2*1000);
    // robot->GetServoCtrl()->Diagnostics();
    // utils::delay_ms(3*1000);

#define ENABLE_ROS

#ifdef ENABLE_ROS
    ros::init(argc, argv, "robodog_hw");

    utils::delay_ms(3000); // Wait until all publisher nodes will launch

    ros::NodeHandle n1;
    ros::ServiceServer s1 = n1.advertiseService("gcode", gcode_callback);

    ros::NodeHandle n2;
    ros::ServiceServer s2 = n2.advertiseService("say", say_callback);

    RemoteController rc(robot.get());
    if (!rc.open("/dev/ttyUSB1")) return -1; // Dev
    //if (!rc.open("/dev/ttyAMA0")) return -1; // PI3
    //if (!rc.open("/dev/ttyS0")) return -1; // PI4
    rc.run();

    ROS_INFO("Robodog HW is running...");

    //g_ctrl->run_from_file("/home/ubuntu/catkin_ws/src/robodog/GCODE/test.gcode");

    ros::spin();
#else
    //g_ctrl->run_from_file("src/robodog/GCODE/test.gcode");
#endif

    robot->GetServoCtrl()->Stop();

    return 1;
}
