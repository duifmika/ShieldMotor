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


void MotionControl::applyCorrection(double leftCm, double rightCm) {
    const float Kp_pos = 4.5;
    const float Kd_pos = 2.5;
    const float Kp_yaw = 3.8;
    const float Kd_yaw = 2.2;

    const int baseSpeed = 255;
    const int controlInterval = 50; 

    static float centerDeadband = 1.0; 

    // PID state
    static float prev_error_pos = 0;
    static float prev_error_yaw = 0;
    static float prev_dLeft = 0;
    static float prev_dRight = 0;
    static unsigned long lastTime = 0;

    if (millis() - lastTime < controlInterval) {
        return;
    }

    // for now I added back the offset.
    leftCm += 2.5;
    rightCm += 2.5;

    lastTime = millis();

    bool trustLeft = (leftCm <= 12.0);
    bool trustRight = (rightCm <= 12.0);

    float correction_pos = 0.0;
    float correction_yaw = 0.0;
    float pos_error = 0.0;
    float yaw_error = 0.0;
    
    if (trustLeft || trustRight) {
        // === POSITION ERROR ===
        pos_error = (leftCm - rightCm) / 2.0;

        if (abs(pos_error) < centerDeadband) pos_error = 0;

        float derivative_pos = (pos_error - prev_error_pos) / (controlInterval / 1000.0);
        correction_pos = Kp_pos * pos_error + Kd_pos * derivative_pos;
        correction_pos = constrain(correction_pos, -55.0, 55.0);
        prev_error_pos = pos_error;

        // === YAW ERROR ===
        float delta_dLeft = leftCm - prev_dLeft;
        float delta_dRight = rightCm - prev_dRight;
        yaw_error = delta_dLeft - delta_dRight;

        if (abs(yaw_error) < 0.5) yaw_error = 0;

        float derivative_yaw = (yaw_error - prev_error_yaw) / (controlInterval / 1000.0);
        correction_yaw = Kp_yaw * yaw_error + Kd_yaw * derivative_yaw;
        correction_yaw = constrain(correction_yaw, -55.0, 55.0);
        prev_error_yaw = yaw_error;

        prev_dLeft = leftCm;
        prev_dRight = rightCm;
    } else {
        // No trustable wall → reset previous errors
        prev_error_pos = 0;
        prev_error_yaw = 0;
    }

    // === DYNAMIC SPEED ADJUSTMENT ===
    float correctionMagnitude = abs(correction_pos) + abs(correction_yaw);
    int adjustedBaseSpeed = baseSpeed - int(correctionMagnitude * 0.7);
    adjustedBaseSpeed = constrain(adjustedBaseSpeed, 160, baseSpeed);

    int speedLeft = constrain(adjustedBaseSpeed - correction_pos - correction_yaw, 0, 255);
    int speedRight = constrain(adjustedBaseSpeed + correction_pos + correction_yaw, 0, 255);

    if (m_driveDir == BACKWARD) {
        // invert movement when going backward
        int tmp = speedLeft;
        speedLeft = speedRight;
        speedRight = tmp;
    }

    m_frontLeft->setSpeed(speedLeft);
    m_backLeft->setSpeed(speedLeft);
    m_frontRight->setSpeed(speedRight);
    m_backRight->setSpeed(speedRight);
}

void MotionControl::drive(double fromX, double fromY, int8_t toX, int8_t toY, double leftCm, double rightCm, double centerCm) {
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
        return;
    }

    if (deltaHeading > 3.0/4.0*PI) {
        goBackward();
        applyCorrection(leftCm, rightCm);
        return;
    }

    goForward();
    applyCorrection(leftCm, rightCm);
}

