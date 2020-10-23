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
#include <sys/wait.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Utils.h"
#include "SoundService.h"

void play_sound(const std::string& file)
{
    auto str = utils::printf("play -q %s &", file.c_str());
    std::system(str.c_str());
}

bool say(robodog::SayService::Request& req, robodog::SayService::Response& res)
{
    const char* lang = req.lang.c_str();
    if (strlen(lang) == 0)
        lang = "en";

    const char* opt = req.options.c_str();
    if (strlen(opt) == 0)
    {
        if (strcmp(lang, "ru") == 0)
            opt = "--pitch 0.3 --scale 60000";
        else
            opt = "--scale 50000";
    }

    //sprintf(cmd, "pico2wave -w 1.wav '%s' && play -q 1.wav speed 1 treble 30 gain 10", msg);
    auto str = utils::printf("gtts-cli -l %s '%s' | mpg123 -q %s - &", lang, req.text.c_str(), opt);
    std::system(str.c_str());

    return true;
}

bool say_callback(robodog::SayService::Request& req, robodog::SayService::Response& res)
{
    ROS_INFO("Received say text: [%s]", req.text.c_str());

    return say(req, res);
}
