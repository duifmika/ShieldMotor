#include "MotionControl.h"
#include <math.h>

void MotionControl::init(double cellWidth, double wallWidth, 
            ShieldMotor* frontLeft, ShieldMotor* frontRight, 
            ShieldMotor* backLeft, ShieldMotor* backRight) {
    
    m_cellWidth = cellWidth;
    m_wallWidth = wallWidth;

    m_frontLeft = frontLeft; 
    m_frontRight = frontRight;
    m_backLeft = backLeft;
    m_backRight = backRight;
}

double MotionControl::getHeading() const {
    return m_heading;
}

double MotionControl::calculateHeading(double fromX, double fromY, uint8_t toX, uint8_t toY) const {
    return atan2(toY-fromY, toX-fromX) + 0.5*PI;
}

double MotionControl::calculateDistance(double fromX, double fromY, uint8_t toX, uint8_t toY) const {
    return abs((m_cellWidth+m_wallWidth)*(toX-fromX + toY-fromY));
}

bool MotionControl::rotateTo(double heading) {
    // TODO: make this (first)
    // Use motors like this:
    // m_frontLeft->run(FORWARD, 255);
    return false;
}

bool MotionControl::drive(double fromX, double fromY, uint8_t toX, uint8_t toY) {
    // TODO: make this (second)
    return false;
}
