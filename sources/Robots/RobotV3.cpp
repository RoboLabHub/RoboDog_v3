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
#include "IK.h"
#include "Utils.h"
#include "Factory.h"
#include "Servos/ServoController.h"

#include <math.h>

const double kDegToServo = 1.0 / 0.24;
const double kWheelRatio = 2.0;
const int    kMaxTemp    = 65;

class RobotV3 : public Robot
{
public:
    virtual bool angle_to_servo_pos(int servoId, double angle, int& pos, bool silent) override
    {
        if (std::isnan(angle)) {
            if (!silent) printf("angle_to_servo_pos(servoId=%d): angle %f is NAN\n", servoId, angle);
            return false;
        }

        switch (servoId) {
            case  1:
            case 11:
                pos = 500 - IK::toDeg(angle) * kDegToServo * kWheelRatio;
                break;
            case  2:
            case 12:
                // 500 = X - IK::toDeg(PI/2) * kDegToServo = X - 375  ==>  X = 875
                pos = 875 - IK::toDeg(angle) * kDegToServo;
                break;
            case  3:
            case 13:
                angle_to_servo_pos(2, angle, pos, silent); pos = 1000 - pos;
                break;
            case  4:
            case 14:
                // Lower leg has offset, so X=500 is not equal to 0 deg  ==>  use X=600
                pos = 600 + IK::toDeg(PI/2 - angle) * kDegToServo;
                break;
            case  5:
            case 15:
                angle_to_servo_pos(4, angle, pos, silent); pos = 1000 - pos;
                break;

            case  6:
            case 16:
                pos = 500 + IK::toDeg(angle) * kDegToServo * kWheelRatio;
                break;
            case  7:
            case 17:
                // 500 = X + IK::toDeg(PI/2) * kDegToServo = X + 375  ==>  X = 125
                pos = 125 + IK::toDeg(angle) * kDegToServo;
                break;
            case  8:
            case 18:
                angle_to_servo_pos(7, angle, pos, silent); pos = 1000 - pos;
                break;
            case  9:
            case 19:
                // Lower leg has offset, so X=500 is not equal to 0 deg  ==>  use X=400
                pos = 400 - IK::toDeg(PI/2 - angle) * kDegToServo;
                break;
            case 10:
            case 20:
                angle_to_servo_pos(9, angle, pos, silent); pos = 1000 - pos;
                break;

            default:
                printf("angle_to_servo_pos(): Unknown servo id %d\n", servoId);
                return false;
        }

        //silent = false;
        if (pos < 0) {
            if (!silent) printf("Servo %d out of range (%d), adjust to 0\n", servoId, pos);
            pos = 0;
            return false;
        }
        else if (pos > 1000) {
            if (!silent) printf("Servo %d out of range (%d), adjust to 1000\n", servoId, pos);
            pos = 1000;
            return false;
        }

        return true;
    }

    virtual GCodeController* GetGcodeCtrl() override { return m_gcodeCtrl.get(); }
    virtual ServoController* GetServoCtrl() override { return m_servoCtrl.get(); }

    virtual int GetVer() override { return 3; }

public:
    std::shared_ptr<GCodeController> m_gcodeCtrl;
    std::shared_ptr<ServoController> m_servoCtrl;
};

namespace Factory
{
    bool CreateRobotV3(std::shared_ptr<Robot>& robot)
    {
        auto lx16a = std::make_shared<LX16a>();
        if (!lx16a->open("/dev/ttyUSB0")) return false;

        auto robotV3 = std::make_shared<RobotV3>();

        // Create servo with offsets
        auto s1_1 = Factory::CreateLX16a(1, lx16a, -10);
        auto s1_2 = Factory::CreateLX16a(2, lx16a, -10);
        auto s1_3 = Factory::CreateLX16a(3, lx16a,  45);
        auto s1_4 = Factory::CreateLX16a(4, lx16a, -35);
        auto s1_5 = Factory::CreateLX16a(5, lx16a,  -5);

        auto s2_1 = Factory::CreateLX16a( 6, lx16a,  20);
        auto s2_2 = Factory::CreateLX16a( 7, lx16a,  45);
        auto s2_3 = Factory::CreateLX16a( 8, lx16a, -10);
        auto s2_4 = Factory::CreateLX16a( 9, lx16a, -10);
        auto s2_5 = Factory::CreateLX16a(10, lx16a, -60);

        auto s3_1 = Factory::CreateLX16a(11, lx16a,  90);
        auto s3_2 = Factory::CreateLX16a(12, lx16a, -10);
        auto s3_3 = Factory::CreateLX16a(13, lx16a,  12);
        auto s3_4 = Factory::CreateLX16a(14, lx16a,   0);
        auto s3_5 = Factory::CreateLX16a(15, lx16a,   0);

        auto s4_1 = Factory::CreateLX16a(16, lx16a,   0);
        auto s4_2 = Factory::CreateLX16a(17, lx16a,  20);
        auto s4_3 = Factory::CreateLX16a(18, lx16a, -15);
        auto s4_4 = Factory::CreateLX16a(19, lx16a,   0);
        auto s4_5 = Factory::CreateLX16a(20, lx16a,  20);

        std::vector<std::shared_ptr<Servo>> servos;

        servos.push_back(s1_1);
        servos.push_back(s1_2);
        servos.push_back(s1_3);
        servos.push_back(s1_4);
        servos.push_back(s1_5);

        servos.push_back(s2_1);
        servos.push_back(s2_2);
        servos.push_back(s2_3);
        servos.push_back(s2_4);
        servos.push_back(s2_5);

        servos.push_back(s3_1);
        servos.push_back(s3_2);
        servos.push_back(s3_3);
        servos.push_back(s3_4);
        servos.push_back(s3_5);

        servos.push_back(s4_1);
        servos.push_back(s4_2);
        servos.push_back(s4_3);
        servos.push_back(s4_4);
        servos.push_back(s4_5);

        for (auto& s : servos) {
            s->set_max_temp(kMaxTemp);
        }

        robotV3->m_servoCtrl = std::make_shared<ServoController>();
        robotV3->m_servoCtrl->AddServoGroup(servos);

        std::map<uint, std::shared_ptr<Joint>> leg1_joints;
        leg1_joints[1] = Factory::CreateJointSingle(s1_1,       robotV3);
        leg1_joints[2] = Factory::CreateJointDouble(s1_2, s1_3, robotV3);
        leg1_joints[3] = Factory::CreateJointDouble(s1_4, s1_5, robotV3);

        std::map<uint, std::shared_ptr<Joint>> leg2_joints;
        leg2_joints[1] = Factory::CreateJointSingle(s2_1,       robotV3);
        leg2_joints[2] = Factory::CreateJointDouble(s2_2, s2_3, robotV3);
        leg2_joints[3] = Factory::CreateJointDouble(s2_4, s2_5, robotV3);

        std::map<uint, std::shared_ptr<Joint>> leg3_joints;
        leg3_joints[1] = Factory::CreateJointSingle(s3_1,       robotV3);
        leg3_joints[2] = Factory::CreateJointDouble(s3_2, s3_3, robotV3);
        leg3_joints[3] = Factory::CreateJointDouble(s3_4, s3_5, robotV3);

        std::map<uint, std::shared_ptr<Joint>> leg4_joints;
        leg4_joints[1] = Factory::CreateJointSingle(s4_1,       robotV3);
        leg4_joints[2] = Factory::CreateJointDouble(s4_2, s4_3, robotV3);
        leg4_joints[3] = Factory::CreateJointDouble(s4_4, s4_5, robotV3);

        auto leg1 = Factory::CreateLeg(1, leg1_joints, false);
        auto leg2 = Factory::CreateLeg(2, leg2_joints, true );
        auto leg3 = Factory::CreateLeg(3, leg3_joints, false);
        auto leg4 = Factory::CreateLeg(4, leg4_joints, true );

        leg1->set_geometry(0.029, 0.165, 0.180); // + leg foot ball (20mm radius)
        leg2->set_geometry(0.029, 0.165, 0.180);
        leg3->set_geometry(0.029, 0.165, 0.180);
        leg4->set_geometry(0.029, 0.165, 0.180);

        // In base frame
        leg1->set_leg_location( 0.209, -0.115, 0);
        leg2->set_leg_location( 0.209,  0.115, 0);
        leg3->set_leg_location(-0.209, -0.099, 0);
        leg4->set_leg_location(-0.209,  0.099, 0);

        std::map<uint8_t, std::shared_ptr<Leg>> legs;
        legs[leg1->getId()] = leg1;
        legs[leg2->getId()] = leg2;
        legs[leg3->getId()] = leg3;
        legs[leg4->getId()] = leg4;

        for (auto& l : legs) {
            l.second->init_with_pos(0, 0.1, 0);
        }
        utils::delay_ms(300);

        robotV3->m_gcodeCtrl = Factory::CreateGCodeController(legs, robotV3);

        robot = robotV3;

        return true;
    }
}
