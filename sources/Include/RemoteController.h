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

#include <string>
#include <memory>
#include <atomic>
#include <mutex>

#include <serial/serial.h>
#include <boost/thread/thread.hpp>

#include "Robot.h"
#include "Button.h"
#include "CommandMenu.h"

enum CMD
{
    kCmdBeep  = 0,
    kCmdClear = 1,
    kCmdMsg   = 2,
};

class RemoteController
{
public:
    RemoteController(Robot* robot);
    virtual ~RemoteController();

    bool open(const std::string port, serial::Timeout timeout = serial::Timeout::simpleTimeout(1000));

    void run();

    void send_cmd(uint8_t cmdId, uint8_t* data = nullptr, uint8_t size = 0);
    void send_msg(int x, int y, const char* msg);
    void send_beep(uint8_t beepType);

    std::shared_ptr<Button> m_btnLeft;
    std::shared_ptr<Button> m_btnRight;

    std::shared_ptr<Button> m_btnL1;
    std::shared_ptr<Button> m_btnL2;
    std::shared_ptr<Button> m_btnL3;

    std::shared_ptr<Button> m_btnR1;
    std::shared_ptr<Button> m_btnR2;
    std::shared_ptr<Button> m_btnR3;

    Robot* m_robot;

    void CommThreadBody();
    void GCodeThreadBody();

private:
    bool send_data(uint8_t* buff, int len);

    uint8_t m_recvData[32];
    bool read_data(double timeoutInSec);

    bool read_byte(uint8_t& byte);

    bool send_gcode(std::string gcode);
    void send_message(const char* msg);
    void send_telemetry(bool force = false);

    std::string m_cmd;
    std::mutex m_cmdMutex;

    std::atomic<bool> m_exit;
    boost::thread*    m_commThread;
    boost::thread*    m_gcodeThread;

    bool m_recording;
    bool m_playing;

    std::shared_ptr<serial::Serial> m_serial;
    std::shared_ptr<CommandMenu>    m_menu;
};
