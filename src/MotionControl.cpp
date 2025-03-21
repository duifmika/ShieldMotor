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

double MotionControl::calculateHeading(double fromX, double fromY, int8_t toX, int8_t toY) const {
    return atan2(toY-fromY, toX-fromX) + 0.5*PI;
}

double MotionControl::calculateDistance(double fromX, double fromY, int8_t toX, int8_t toY) const {
    double deltaX = toX - fromX;
    double deltaY = toY - fromY;
    return sqrt(deltaX * deltaX + deltaY * deltaY) * (m_cellWidth + m_wallWidth);
}

void MotionControl::drive(double fromX, double fromY, int8_t toX, int8_t toY) {
    // update m_carRotation and m_heading
    double distCm = calculateDistance(fromX, fromY, toX, toY);

    m_heading = calculateHeading(fromX, fromY, toX, toY);
    if (fabs(m_heading - m_carRotation) > 0.01) {
        // if dist is very low (the car has arrived) stop the car if in motion
        // Adjust the 3 cm dist if it doesnt work
        if (distCm < 3 && m_frontLeft->getSpeed() > 0) {
            // Stop the car
        }

        // rotate 90 degrees to the right heading 

        m_carRotation = m_heading;
        return;
    }

    // correction

    // go forward

}
