#include "MotionControl.h"
#include <math.h>

void MotionControl::goForward() {
    if (m_driveDir != FORWARD)
        goBrake();

    m_frontLeft->run(FORWARD);
    m_frontRight->run(FORWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(FORWARD);

    m_frontLeft->setSpeed(255);
    m_frontRight->setSpeed(255);
    m_backLeft->setSpeed(255);
    m_backRight->setSpeed(255);   
 
    m_driveDir = FORWARD;
}

void MotionControl::goBackward(){
    if (m_driveDir != BACKWARD)
        goBrake();

    m_frontLeft->run(BACKWARD);
    m_frontRight->run(BACKWARD);
    m_backLeft->run(BACKWARD);
    m_backRight->run(BACKWARD); 

    m_frontLeft->setSpeed(255);
    m_frontRight->setSpeed(255);
    m_backLeft->setSpeed(255);
    m_backRight->setSpeed(255);   
 
    m_driveDir = BACKWARD;
}

void MotionControl::goLeft(){
    if (m_driveDir != RELEASE)
        goBrake();

    m_frontLeft->run(BACKWARD);
    m_frontRight->run(FORWARD);
    m_backLeft->run(BACKWARD);
    m_backRight->run(FORWARD);   
    
    m_frontLeft->setSpeed(150);
    m_frontRight->setSpeed(150);
    m_backLeft->setSpeed(150);
    m_backRight->setSpeed(150);   

    delay(450);
    goBrake();
}

void MotionControl::goRight(){
    if (m_driveDir != RELEASE)
        goBrake();

    m_frontLeft->run(FORWARD);
    m_frontRight->run(BACKWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(BACKWARD); 

    m_frontLeft->setSpeed(150);
    m_frontRight->setSpeed(150);
    m_backLeft->setSpeed(150);
    m_backRight->setSpeed(150);   

    delay(450);
    goBrake();
}

void MotionControl::goBrake(int delayMs){
    if (m_frontLeft->getSpeed() == 0) {
        return;
    }

    m_frontLeft->run(RELEASE);
    m_frontRight->run(RELEASE);
    m_backLeft->run(RELEASE);
    m_backRight->run(RELEASE);

    delay(delayMs); 
    m_driveDir = RELEASE;
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

void MotionControl::applyCorrection(double leftCm, double rightCm, double centerCm) {
    // make left or right wheels go slower
}

bool MotionControl::drive(double fromX, double fromY, int8_t toX, int8_t toY, double leftCm, double rightCm, double centerCm) {
    m_heading = calculateHeading(fromX, fromY, toX, toY);
    double deltaHeading = fabs(m_heading - m_carRotation);
    if (deltaHeading > 0.1 && deltaHeading < (3.0/4.0*PI)) {
      double diff = (m_heading - m_carRotation) / PI;
      if (fabs(fabs(diff) - 1.5) < 0.0001) {
        diff < -1.49 ?  goRight() : goLeft();
      } else if (diff > 0) {
        goRight();
      } else if (diff < 0) {
        goLeft();
      }
        m_carRotation = m_heading;
        return false;
    }

    if (deltaHeading > 3.0/4.0*PI) {
        goBackward();
        applyCorrection(leftCm, rightCm, centerCm);
        // add position tracking
        return false;
    }

    goForward();
    applyCorrection(leftCm, rightCm, centerCm);
    // add position tracking
    return false;
}

