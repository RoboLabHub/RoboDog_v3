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
#include <serial/impl/unix.h>

#include "ros/ros.h"
#include "robodog/GcodeService.h"

#include "Utils.h"
#include "Filters.h"
#include "MathHelpers.h"
#include "RemoteController.h"
#include "GCodeController.h"

using namespace std;

RemoteController::RemoteController(Robot* robot) :
    m_robot(robot), m_exit(false), m_recording(false), m_playing(false),
    m_commThread(nullptr), m_gcodeThread(nullptr)
{
    m_menu = std::make_shared<CommandMenu>(this);

    m_btnLeft  = std::make_shared<Button>(0b00000001);
    m_btnRight = std::make_shared<Button>(0b00000010);

    m_btnL1 = std::make_shared<Button>(0b00000100);
    m_btnL2 = std::make_shared<Button>(0b00001000);
    m_btnL3 = std::make_shared<Button>(0b00010000);

    m_btnR1 = std::make_shared<Button>(0b00100000);
    m_btnR2 = std::make_shared<Button>(0b01000000);
    m_btnR3 = std::make_shared<Button>(0b10000000);
}

RemoteController::~RemoteController()
{
    m_exit.store(true);
    if (m_commThread) m_commThread ->join();
    if (m_gcodeThread) m_gcodeThread->join();
}

bool RemoteController::open(const std::string port, serial::Timeout timeout)
{
    try {
        m_serial = std::make_shared<serial::Serial>(port, 115200, timeout);
        return m_serial->isOpen();
    }
    catch (serial::PortNotOpenedException ex) {
        cout << "serial::PortNotOpenedException: " << port.c_str() << "\n";
    }
    catch (serial::IOException ex) {
        cout << "serial::IOException: " << port.c_str() << "\n";
    }

    return false;
}

bool RemoteController::read_byte(uint8_t& byte)
{
    try {
        m_serial->read(&byte, 1);
    }
    catch (serial::PortNotOpenedException ex) {
        cout << "serial::PortNotOpenedException\n";
        return false;
    }
    catch (serial::IOException ex) {
        cout << "serial::IOException\n";
        return false;
    }
    catch (serial::SerialException ex) {
        cout << "serial::SerialException\n";
        return false;
    }

    return true;
}

static uint8_t GetCheckSum(uint8_t* buff, int len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; ++i) checksum += buff[i];
    return checksum;
}

bool RemoteController::send_data(uint8_t* buff, int len)
{
    try {
        // // Clear input buffer
        // while (m_serial->available()) {
        //     uint8_t byte;
        //     read_byte(byte);
        // }

        return (m_serial->write(buff, len) == len);
    }
    catch (serial::PortNotOpenedException ex) {
        cout << "serial::PortNotOpenedException\n";
    }
    catch (serial::IOException ex) {
        cout << "serial::IOException\n";
    }
    catch (serial::SerialException ex) {
        cout << "serial::SerialException\n";
    }

    return false;
}

struct DATA_PACKET_1
{
  uint8_t type;

  int8_t x1;
  int8_t y1;
  int8_t x2;
  int8_t y2;

  uint8_t buttons;
};

bool RemoteController::read_data(double timeoutInSec)
{
    auto endTime = ros::Time::now() + ros::Duration(timeoutInSec);

    int pos = 0;
    int len, recv_pos, checksum;
    while (false == m_exit.load()) {
        uint8_t byte;
        if (m_serial->available() > 0) {
            if (!read_byte(byte)) {

                cout << "read_byte() failed\n";
                return false;
            }
        }
        else {
            if (ros::Time::now() > endTime) {
                cout << "read_data() timeout\n";
                return false;
            }
            usleep(100); // 0.1 ms
            continue;
        }

        //cout << "byte: " << byte << "\n";

        switch (pos) {
        case 0:
            if (byte != '*') {
                cout << "Bad * header: " << (int)byte << "\n";
                pos = -1;
            }
            break;
        case 1:
            len = byte;
            recv_pos = 0;
            if (len > sizeof(m_recvData)) {
                cout << "Data size " << (int)len << " too big\n";
                pos = -1;
            }
            break;
        case 2:
            checksum = byte;
            break;
        default:
            if (recv_pos < len) {
                m_recvData[recv_pos] = byte;
                if (++recv_pos == len) {
                    auto cs = GetCheckSum(m_recvData, len);
                    if (cs != checksum) {
                        cout << "Bad checksum\n";
                        return false;

                    }
                    return true;
                }
            }
        }
        pos++;
    }

    return false;
}

void RemoteController::send_cmd(uint8_t cmdId, uint8_t* data, uint8_t size)
{
    uint8_t checkSum = GetCheckSum(data, size);

    uint8_t buff[4] = { '*', size, checkSum, cmdId };

    if (!send_data(buff,    4)) cout << "send_cmd() failed 1\n";
    if (!send_data(data, size)) cout << "send_cmd() failed 2\n";
}

void RemoteController::send_msg(int x, int y, const char* msg)
{
    //cout << "send_message: " <<  msg << "\n";

    uint8_t len = strlen(msg);
    if (len > 20) {
        cout << "send_message(): msg too long (>20): %s" << msg << "\n";
        return;
    }

    uint8_t data[32] = {0};
    data[0] = x;
    data[1] = y;
    strcpy((char*)&data[2], msg);

    send_cmd(kCmdMsg, data, len+2);
}

void RemoteController::send_beep(uint8_t beepType)
{
    send_cmd(kCmdBeep, &beepType, 1);
}

bool RemoteController::send_gcode(std::string gcode)
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

void RemoteController::send_message(const char* msg)
{
    cout << "send send_message: " <<  msg << "\n";

    const uint8_t kDataSize = 20;
    uint8_t buff[kDataSize+3+1] = { '*', kDataSize };
    buff[2] = GetCheckSum(&buff[3], kDataSize);
    strncpy((char*)buff+3, msg, 20);
    if (!send_data(buff, kDataSize+3)) cout << "send_message() failed\n";
}

int Exponent(double x, double k = 40)
{
    x /= 100.0;
    double ret = k * (x * x * x); // * sign(x);
    if (abs(ret) < 1) ret = sign(x);
    return ret;
}

void RemoteController::CommThreadBody()
{
    int nPrevMove = 0;
    int rotX = 0;
    int rotY = 0;
    int rotZ = 0;
    int offsetHeight  = 0;
    int offsetSide    = 0;
    int offsetForward = 0;

    int ver = m_robot->GetGcodeCtrl()->GetRobot()->GetVer();

    while (false == m_exit.load()) {

        m_menu->send_telemetry();

        if (!read_data(1.0)) { // Use small timeout to send telemetry faster (for fast RC init) in case of disconnect
            {
                std::lock_guard<std::mutex> guard(m_cmdMutex);
                m_cmd = ""; // Reset last command
            }
            m_menu->send_telemetry(true);
            continue;
        }

        if (m_recvData[0] == 1) { // Check for packet type
            DATA_PACKET_1* data = (DATA_PACKET_1*)&m_recvData;

            //cout << "Buttons: " << (int)data->buttons << "\n";
            //cout << "x1: " << (int)data->x1 << " y1: " << (int)data->y1 << " x2: " << (int)data->x2 << " y2: " << (int)data->y2 << "\n";

            m_btnLeft ->check(data->buttons);
            m_btnRight->check(data->buttons);

            m_btnL1->check(data->buttons);
            m_btnL2->check(data->buttons);
            m_btnL3->check(data->buttons);

            m_btnR1->check(data->buttons);
            m_btnR2->check(data->buttons);
            m_btnR3->check(data->buttons);

            m_menu->process_command();

            std::string cmd;
            bool changedRotOffset = false;
            bool changedAngle     = false;

            bool moveFinished = ros::Time::now() >= (m_robot->GetGcodeCtrl()->GetMoveFinishTime() - ros::Duration(0.2));

            if (m_btnR3->clicked()) cmd = "S 150\n$POSE 3\n";
            else if (m_btnR2->clicked()) cmd = "S 150\n$POSE 4\n";
            else if (m_btnR1->clicked()) {
                cmd = "S 150\n$POSE 0\nW 500\nSERVO_OFF\n";
                m_recording = m_playing = false;
            }
            else if (m_btnL3->double_clicked()) {
                send_message("Recording...");

                m_recording = true;
            }
            else if (m_btnL2->double_clicked()) {
                send_message("Plaing...");
                m_playing = true;
            }
            else if (m_btnLeft->clicked()) {
                rotX = rotY = rotZ = 0;
                offsetForward = offsetSide = offsetHeight = 0;
                changedRotOffset = true;
            }
            else if (m_btnL1->on()) {
                rotY = constrain(rotY - data->y2/20, -45, 45);
                rotZ = constrain(rotZ + data->x2/20, -45, 45);
                if (m_btnRight->clicked()) rotY = rotZ = 0;

                changedRotOffset = true;
            }
            else if (m_btnL2->on()) {
                rotX          = constrain(rotX          - data->x2/20,  -45 , 45);
                offsetForward = constrain(offsetForward + data->y2/10, -100, 100);
                if (m_btnRight->clicked()) rotX = offsetForward = 0;
                changedRotOffset = true;
            }
            else if (m_btnL3->on()) {
                offsetSide    = constrain(offsetSide    - data->x2/10,  -50,  50);
                offsetHeight  = constrain(offsetHeight  - data->y2/ 5, -145, 150);
                if (m_btnRight->clicked()) offsetSide = offsetHeight = 0;
                changedRotOffset = true;
            }
            else if (!m_btnR2->on() && abs(data->y2) > 5) {
                rotY  = constrain(rotY - data->y2/20, -30, 30);
                changedAngle = true;
            }
            else if (m_btnRight->clicked()) {
                rotY = 0;
                changedAngle = true;
            }

            if (changedRotOffset) {
                cmd += utils::printf("BASE_ROT %d %d %d\n", rotX, rotY, rotZ);
                cmd += utils::printf("BASE_OFFSET %d %d %d\n", offsetForward, offsetSide, offsetHeight);
                cmd += utils::printf("S 100\n$SET_POSE 0 0 0\n");
            }
            else {

                if (abs(data->x1) > 5 || abs(data->y1) > 5 || abs(data->x2) > 5 || m_btnR2->on()) {
                    // int nMoveDir = (data->y1 > 0) ? 1 : 0;
                    // if (nMoveDir != nPrevMove) {
                    //     // Set forward offset depends of move direction
                    //     cmd += utils::printf("S 200\n$SET_POSE %d 0 %d\n", nMoveDir ? 0 : 30, offsetHeight);

                    //     nPrevMove = nMoveDir;
                    // }

                    if (m_btnR2->on()) {
                        if (abs(data->y2) > 10) {
                            int height  = abs(data->y2) / 2;
                            int forward = -data->y1 / 5;
                            int side    =  data->x1 / 5;
                            offsetForward = offsetSide = 0;
                            cmd += utils::printf("BASE_OFFSET %d %d %d\n", offsetForward, offsetSide, offsetHeight);
                            cmd += utils::printf("$JUMP_2 %d %d %d\n", height, forward, side);
                        }
                    }
                    else {
                        int forward_step = data->y1 /  2;
                        int side_step    = data->x2 /  4;
                        int rotate_step  = data->x1 / 20;

                        if (m_robot->GetVer() == 2) {
                            forward_step /= 1.5;
                            side_step    /= 1.5;
                        }

                        if (abs(rotate_step) < 5) rotate_step = 5 * sign(rotate_step);

                        int offsetForwardCorr = 40;
                        if (data->y1 < 0) offsetForwardCorr = 30; // Backward move

                        //offsetForwardCorr -= rotY;

                        int rot = rotX + (side_step / 3);
                        cmd += utils::printf("BASE_ROT %d %d %d\n", rot, rotY, rotate_step);
                        cmd += utils::printf("BASE_OFFSET %d %d %d\n", offsetForward + offsetForwardCorr, offsetSide, offsetHeight);
                        cmd += utils::printf("$WALK_4 %d %d\n", forward_step, side_step);
                    }
                }
                else if (changedAngle) {
                    if (moveFinished) {
                        cmd += utils::printf("BASE_ROT %d %d %d\n", rotX, rotY, rotZ);
                        cmd += utils::printf("BASE_OFFSET %d %d %d\n", offsetForward, offsetSide, offsetHeight);
                        cmd += utils::printf("S 100\n$SET_POSE 0 0 0\n");
                    } else {
                        std::lock_guard<std::mutex> guard(m_cmdMutex);
                        m_cmd = utils::printf("BASE_ROT %d %d %d\n", rotX, rotY, rotZ);
                    }
                }
            }

            if (moveFinished) {
                std::lock_guard<std::mutex> guard(m_cmdMutex);
                m_cmd = cmd;

                // Give a change for GCodeThreadBody to catch the cmd
                //usleep(10*1000); // 10 ms
            }
        }

        // Sleep to avoid 100% CPU usage
        usleep(1000); // 1 ms
    }

    cout << "CommThreadBody exit\n";
}

void RemoteController::GCodeThreadBody()
{
    while (false == m_exit.load()) {
        std::string cmd;
        {
            std::lock_guard<std::mutex> guard(m_cmdMutex);
            cmd = m_cmd;
            m_cmd = "";
        }

        if (cmd.length() > 0) {
            //cout << cmd;
            send_gcode(cmd);
        }
        else
            usleep(100); // 100 ms
    }

    cout << "GCodeThreadBody exit\n";
}

void RemoteController::run()
{
    m_commThread  = new boost::thread(boost::bind(&RemoteController::CommThreadBody,  this));
    m_gcodeThread = new boost::thread(boost::bind(&RemoteController::GCodeThreadBody, this));
}
