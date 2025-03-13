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

double MotionControl::getCarRotation() const {
    return m_carRotation;
}

double MotionControl::calculateHeading(double fromX, double fromY, uint8_t toX, uint8_t toY) const {
    return atan2(toY-fromY, toX-fromX) + 0.5*PI;
}

double MotionControl::calculateDistance(double fromX, double fromY, uint8_t toX, uint8_t toY) const {
    double deltaX = toX - fromX;
    double deltaY = toY - fromY;
    return sqrt(deltaX * deltaX + deltaY * deltaY) * (m_cellWidth + m_wallWidth);
}

bool MotionControl::rotateTo(double heading) {
    // Use motors like this:
    // m_frontLeft->run(FORWARD, 255);
    return false;
}

bool MotionControl::drive(double fromX, double fromY, uint8_t toX, uint8_t toY) {
    // update m_carRotation and m_heading
    return false;
}
