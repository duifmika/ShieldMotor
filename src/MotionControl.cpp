#include "MotionControl.h"
#include <math.h>

void MotionControl::goForward(){
    m_frontLeft->run(FORWARD);
    m_frontRight->run(FORWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(FORWARD);
}

void MotionControl::goLeft(){
    m_frontLeft->run(BACKWARD);
    m_frontRight->run(FORWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(BACKWARD);   
}

void MotionControl::goBackward(){
    m_frontLeft->run(BACKWARD);
    m_frontRight->run(BACKWARD);
    m_backLeft->run(BACKWARD);
    m_backRight->run(BACKWARD); 
}

void MotionControl::goRight(){
    m_frontLeft->run(FORWARD);
    m_frontRight->run(BACKWARD);
    m_backLeft->run(BACKWARD);
    m_backRight->run(FORWARD); 
}

void MotionControl::goBrake(){
    m_frontLeft->run(RELEASE);
    m_frontRight->run(RELEASE);
    m_backLeft->run(RELEASE);
    m_backRight->run(RELEASE);
}

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

CompassDirMC MotionControl::radiansToDirection(double angleRad) const {
    angleRad = fmod(angleRad, 2 * PI);
    if (angleRad < 0) angleRad += 2*PI;

    if (angleRad >= 7.0 / 4.0 * PI || angleRad < 1.0 / 4.0 * PI) return CompassDirMC::North;
    if (angleRad >= 1.0 / 4.0 * PI && angleRad < 3.0 / 4.0 * PI) return CompassDirMC::East;
    if (angleRad >= 3.0 / 4.0 * PI && angleRad < 5.0 / 4.0 * PI) return CompassDirMC::South;
    return CompassDirMC::West;
}

void MotionControl::drive(double fromX, double fromY, int8_t toX, int8_t toY) {
    // update m_carRotation and m_heading
    double distCm = calculateDistance(fromX, fromY, toX, toY);

    m_heading = calculateHeading(fromX, fromY, toX, toY);
    if (fabs(m_heading - m_carRotation) > 0.01) {
        // if dist is very low (the car has arrived) stop the car if in motion
        // Adjust the 3 cm dist if it doesnt work
        if (distCm < 3 && m_frontLeft->getSpeed() > 0) {
            goBrake();
            delay(100); // remtijd bepalen
        }

        // rotate 90 degrees to the right heading 

        m_carRotation = m_heading;
        return;
    }

    if (m_frontLeft->getSpeed() == 0) {
        double offX = fromX - floor(fromX);
        double offY = fromY - floor(fromY);

        m_frontLeft->setSpeed(150);
        m_frontRight->setSpeed(150);
        m_backLeft->setSpeed(150);
        m_backRight->setSpeed(150); 

        if (radiansToDirection(m_carRotation) == CompassDirMC::North){
            if (offX >= 0.05) {
                goLeft();
            }
            else if(offX <= -0.05) {
                goRight();
            }

        }

        if (radiansToDirection(m_carRotation) == CompassDirMC::South){
            if (fabs(offX) >= 0.05) {
                goRight();
            }
            else if(fabs(offX) <= -0.05){
                goLeft();
            }
        }

        if (radiansToDirection(m_carRotation) == CompassDirMC::East){
            if (fabs(offY) >= 0.05) {
                goLeft();
            }
            else if(fabs(offY) <= -0.05) {
                goRight();
            }
        }

        if (radiansToDirection(m_carRotation) == CompassDirMC::West){
            if (fabs(offY) >= 0.05) {
                goRight();
            }
            else if(fabs(offY) <= -0.05) {
                goLeft();
            }
        }

        delay(50) // small adjustment time
        goBrake();
    }

    goForward();

}
