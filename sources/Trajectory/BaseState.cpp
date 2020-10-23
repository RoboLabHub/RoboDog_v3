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
#include <math.h>
#include <string.h>

#include "Utils.h"
#include "BaseState.h"
#include "GCodeController.h"

BaseState::BaseState(GCodeController* gcodeCtrl) :
    m_gcodeCtrl(gcodeCtrl), m_moveTime(0)
{
    Init();
}

bool BaseState::Init()
{
    for (int i = 0; i < 4; ++i) m_legToAdjust[i] = true;

    if (!GetCurrentPos()) return false;
    return true;
}

int BaseState::GetMoveTimeInMS(IK::Vector v, double speed)
{
    double dist = sqrt(v.x*v.x + v.y*v.y + v.z*v.z); // in meters
    m_moveTime = (dist / speed) * 1000;

    return m_moveTime;
}

IK::Vector BaseState::GetMoveVector(int legId, IK::Vector pos)
{
    IK::Vector v;

    v.x = m_legAdjusted[legId-1].pos.x - pos.x;
    v.y = m_legAdjusted[legId-1].pos.y - pos.y;
    v.z = m_legAdjusted[legId-1].pos.z - pos.z;

    return v;
}

IK::Vector BaseState::GetPosByVector(int legId, IK::Vector v)
{
    IK::Vector pos;

    pos.x = m_leg[legId-1].pos.x + v.x;
    pos.y = m_leg[legId-1].pos.y + v.y;
    pos.z = m_leg[legId-1].pos.z + v.z;

    return pos;
}

bool BaseState::MoveLeg(int legId, IK::Vector pos, double speed)
{
    IK::Vector p = GetAdjustedLegPosition(legId, pos);
    IK::Vector v = GetMoveVector(legId, p);
    m_moveTime = GetMoveTimeInMS(v, speed);

    m_leg        [legId-1].pos = pos;
    m_legAdjusted[legId-1].pos = p;

    return m_gcodeCtrl->add_point(legId, p, m_moveTime);
}

bool BaseState::MoveBase(IK::Vector v, double speed)
{
    for (int legId = 1; legId <= 4; ++legId) {
        IK::Vector pos = GetPosByVector(legId, v);

        if (!MoveLeg(legId, pos, speed)) return false;
    }
    return true;
}

bool BaseState::SetBase(IK::Vector pos, double speed)
{
    for (int legId = 1; legId <= 4; legId++) {
        if (!MoveLeg(legId, pos, speed)) return false;
    }
    return true;
}

IK::Vector BaseState::GetLegPositionInBaseFrame(int legId, IK::Vector pos)
{
    // Base axises: X along base length (from robot center to forward), Y along base width (from robot center to left), Z is height (up)

    // Convert axises from leg frame to base frame
    IK::Vector posInBaseFrame;
    posInBaseFrame.x =  1 * pos.z;
    posInBaseFrame.y =  1 * pos.x;
    posInBaseFrame.z = -1 * pos.y;

    auto legs = m_gcodeCtrl->GetLegs();
    auto loc = legs[legId]->get_leg_location();

    posInBaseFrame.x += loc.x;
    posInBaseFrame.y += loc.y;
    posInBaseFrame.z += loc.z;

    return posInBaseFrame;
}

IK::Vector BaseState::GetLegPositionInLegFrame(int legId, IK::Vector pos)
{
    // Leg axises: X from leg's center to robot's left, Y from legs' center to down, Z from leg's center to robot forward

    auto legs = m_gcodeCtrl->GetLegs();
    auto loc = legs[legId]->get_leg_location();

    // Convert axises from base frame to leg frame
    IK::Vector posInLegFrame;
    posInLegFrame.x =  1 * (pos.y - loc.y);
    posInLegFrame.y = -1 * (pos.z - loc.z);
    posInLegFrame.z =  1 * (pos.x - loc.x);

    return posInLegFrame;
}

// Adjust by base rotation (m_rot) and offset (m_baseOffset)
IK::Vector BaseState::GetAdjustedLegPosition(int legId, IK::Vector pos)
{
    IK::Vector legRoot;
    legRoot.x = pos.x;
    legRoot.y = pos.y;
    legRoot.z = pos.z;

    IK::Vector legInBaseFrame = GetLegPositionInBaseFrame(legId, legRoot);

    IK::Vector legPos = m_legToAdjust[legId-1] ? IK::rotate(legInBaseFrame, m_rot) : legInBaseFrame;

    if (m_legToAdjust[legId-1]) {
        legPos.x -= m_baseOffset.x;
        legPos.y -= m_baseOffset.y;
        legPos.z += m_baseOffset.z;
    }

    return GetLegPositionInLegFrame(legId, legPos);
}

void BaseState::RotateInBase(int legId, double& x, double& y, double& z, IK::Vector rot)
{
    IK::Vector pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    IK::Vector legInBaseFrame = GetLegPositionInBaseFrame(legId, pos);

    IK::Vector legPos = IK::rotate(legInBaseFrame, rot);

    IK::Vector adjusted = GetLegPositionInLegFrame(legId, legPos);

    x = adjusted.x;
    y = adjusted.y;
    z = adjusted.z;
}

void BaseState::AdjustByOffset(int legId, double& x, double& y, double& z, bool adjustX, bool adjustY, bool adjustZ)
{
    IK::Vector pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    IK::Vector legInBaseFrame = GetLegPositionInBaseFrame(legId, pos);

    if (adjustX) legInBaseFrame.x -= m_baseOffset.x;
    if (adjustY) legInBaseFrame.y -= m_baseOffset.y;
    if (adjustZ) legInBaseFrame.z += m_baseOffset.z;

    IK::Vector adjusted = GetLegPositionInLegFrame(legId, legInBaseFrame);

    x = adjusted.x;
    y = adjusted.y;
    z = adjusted.z;
}

void BaseState::WaitForMove(double k)
{
    utils::delay_ms(k * m_moveTime);
}

bool BaseState::GetCurrentPos()
{
    // ToDo: get from servo pos + forward kinematics
    return true;
}
