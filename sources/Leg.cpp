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
#include <map>
#include <math.h>
#include <iterator>
#include <algorithm>
#include <memory>
#include <mutex>

#include "Leg.h"
#include "Joint.h"
#include "IK.h"
#include "Utils.h"

class LegImpl : public Leg
{
public:
    LegImpl(int legId, std::map<uint, std::shared_ptr<Joint>> joints, bool inverseX) :
        m_legId(legId),
        m_joints(joints), m_inverseX(inverseX),
        m_x(0), m_y(0), m_z(0),
        m_x0(0), m_y0(0), m_z0(0)
    {
    }

    virtual int getId() override { return m_legId; }

    virtual void set_leg_location(double x, double y, double z) override
    {
        m_x0 = x;
        m_y0 = y;
        m_z0 = z;
    }

    virtual IK::Vector get_leg_location() override
    {
        IK::Vector vec;
        vec.x = m_x0;
        vec.y = m_y0;
        vec.z = m_z0;
        return vec;
    }

    virtual void set_geometry(double link1, double link2, double link3) override
    {
        m_ik = std::make_shared<IK>(link1, link2, link3);
    }

    virtual bool move(double angles[3], int move_time, bool simulate)
    {
        if (!m_joints[1]->move(angles[0], move_time, simulate)) return false;
        if (!m_joints[2]->move(angles[1], move_time, simulate)) return false;
        if (!m_joints[3]->move(angles[2], move_time, simulate)) return false;

        return true;
    }

    virtual bool move(double x, double y, double z, int move_time, bool simulate) override
    {
        double adjustedX = m_inverseX ? (-x - m_ik->m_L1) : (x - m_ik->m_L1);

        double angles[3];
        m_ik->inverseKinematics(adjustedX, y, z, angles);

        if (!move(angles, move_time, simulate)) {
            if (!simulate) {
                std::cout << "Move failed to " << x << " " << y << " " << z << "\n";
            }
            return false;
        }

        // Save the leg's new position
        if (!simulate) {
            m_x = x;
            m_y = y;
            m_z = z;
        }

        return true;
    }

    virtual bool init_with_pos(double x, double y, double z) override
    {
        m_lastPoint.x = x;
        m_lastPoint.y = y;
        m_lastPoint.z = z;

        bool ret = move(x, y, z, 1000, false);
        return ret;
    }

    virtual bool add_point(double x, double y, double z, const ros::Time& time) override
    {
        //std::cout << "Add point time: " << utils::str(time).c_str() << "\n";

        int count = 0;
        {
            std::lock_guard<std::mutex> guard(m_mutexPoints);
            count = m_points.size();
        }

        if (count > 0) {
            auto t = get_final_point().time;
            if (time < t) {
                std::cout << "Bad point time: " << utils::str(time).c_str() << " (less than final: " << utils::str(t).c_str() << ")\n";
                return false;
            }
        }

        MOVE_POINT p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.time = time;

        {
            std::lock_guard<std::mutex> guard(m_mutexPoints);
            m_points.push_back(p);
        }

        return true;
    }

    virtual MOVE_POINT get_final_point() override
    {
        std::lock_guard<std::mutex> guard(m_mutexPoints);

        if (m_points.size() > 0) {
            auto p = m_points.back();
            //std::cout << "Final point: " << p.x << ", " << p.y << ", " << p.z << " " << utils::str(p.time).c_str() << "\n";
            return p;
        }

        m_lastPoint.time = ros::Time::now();
        //std::cout << "No points left in queue, return m_lastPoint (time=" << m_lastPoint.time << ")\n";
        return m_lastPoint;
    }

    virtual void cycle() override
    {
        if (m_points.size() == 0) return;

        MOVE_POINT p1;
        MOVE_POINT p2;

        ros::Time currTime = ros::Time::now();

        // Find from/to points for current time
        {
            std::lock_guard<std::mutex> guard(m_mutexPoints);

            p1 = m_lastPoint;

            bool deleteFirst = false;
            for (auto& p : m_points) {
                p2 = p;
                if (p2.time >= currTime) break;

                p1 = p2;
                deleteFirst = true;
            }

            if (deleteFirst) {
                //std::cout << "Erase first point in m_points\n";
                m_points.erase(m_points.begin());
            }

            m_lastPoint = p1;

            if (currTime >= p2.time) {
                currTime = p2.time;
                m_points.clear();
                //std::cout << "Clear all m_points\n";
            }
        }

        ros::Duration diff1 = currTime - p1.time;
        ros::Duration diff2 = p2.time  - p1.time;
        double k = 1;
        if (diff2.toSec() != 0) {
            k = diff1.toSec() / diff2.toSec();
            //k = pow(k, 0.5); // To move servo faster
        }
        if (k < 0 || k > 1) {
            std::cout << "Invalid k in leg cycle: " << k << "\n";
            return;
        }

        //std::cout << "k: " << k << "\n";

        double x = p1.x + (k * (p2.x - p1.x));
        double y = p1.y + (k * (p2.y - p1.y));
        double z = p1.z + (k * (p2.z - p1.z));

        move(x, y, z, 0, false);
    }

    // Upper leg down, lower leg forward (parallel to body)
    virtual void tune_pos_1(bool useIK) override
    {
        if (useIK) {
            move(0, m_ik->m_L2, m_ik->m_L3, 1000, false);
        }
        else {
            double angles[] = { 0, 0, PI/2 };
            move(angles, 1000, false);
        }
    }

    // Upper leg backward (parallel to body), lower leg down
    virtual void tune_pos_2(bool useIK) override
    {
        if (useIK) {
            move(0, m_ik->m_L3, -m_ik->m_L2, 1000, false);
        }
        else {
            double angles[] = { 0, PI/2, PI/2 };
            move(angles, 1000, false);
        }
    }

private:
    int m_legId;

    double m_x, m_y, m_z;
    double m_x0, m_y0, m_z0;
    bool m_inverseX;

    std::shared_ptr<IK>  m_ik;
    std::map<uint, std::shared_ptr<Joint>> m_joints;

    MOVE_POINT m_lastPoint;
    std::vector<MOVE_POINT> m_points;
    std::mutex m_mutexPoints;
};

namespace Factory
{
    std::shared_ptr<Leg> CreateLeg(int legId, std::map<uint, std::shared_ptr<Joint>> joints, bool inverseX)
    {
        return std::make_shared<LegImpl>(legId, joints, inverseX);
    }
}
