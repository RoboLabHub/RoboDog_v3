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
#include <sstream>

#include "ros/ros.h"
#include "robodog/GcodeService.h"

bool SendGcode(std::string gcode)
{
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robodog::GcodeService>("gcode");
    robodog::GcodeService srv;
    srv.request.gcode = gcode;
    if (client.call(srv)) {
        //ROS_INFO("Send OK");
    }
    else
        ROS_INFO("Send gcode failed");

    return true;
}

bool SendGcodeFromFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        printf("SendGcodeFromFile(): Failed to open file: %s\n", path.c_str());
        return false;
    }

    std::stringstream strStream;
    strStream << file.rdbuf();

    return SendGcode(strStream.str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robodog_client");
    ros::NodeHandle n;

    SendGcodeFromFile("GCODE/test.gcode");

    return 0;
}
