#include "MotionControl.h"
#include <math.h>
#include <Wire.h>

void MotionControl::goForward() {
    if (m_driveDir != FORWARD)
        goBrake();

    m_frontLeft->run(FORWARD);
    m_frontRight->run(FORWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(FORWARD);

    if (m_frontLeft->getSpeed() == 0) {
        m_frontLeft->setSpeed(255);
        m_frontRight->setSpeed(255);
        m_backLeft->setSpeed(255);
        m_backRight->setSpeed(255);   
    }

    m_driveDir = FORWARD;
}

MPU MotionControl::readMPU() {
    MPU mpu = {0};
    Wire.beginTransmission(0x68);
    Wire.write(0x43);  // Starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);  // Request 6 bytes (3 for gyroscope)

    // Read gyroscope values
    mpu.gx = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H, GYRO_XOUT_L
    mpu.gy = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H, GYRO_YOUT_L
    mpu.gz = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H, GYRO_ZOUT_L

    // Convert raw gyroscope data to degrees per second (assuming ±250°/s range)
    mpu.gx = mpu.gx / 131.0;  // 131 LSB/°/s for ±250°/s range
    mpu.gy = mpu.gy / 131.0;
    mpu.gz = mpu.gz / 131.0;

    static unsigned long lastTime = millis();
    double dt = (millis() - lastTime) / 1000.;
    lastTime = millis();
    if (abs(mpu.gz) > 2. && dt < 0.020)
        m_yaw += mpu.gz*dt;

    return mpu;
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

    delay(m_rotTime + 60);
    m_carRotation -= 0.5*PI;
    goBrake(100, 0);
}

void MotionControl::goRight(){
    if (m_driveDir != RELEASE)
        goBrake();

    m_frontLeft->run(FORWARD);
    m_frontRight->run(BACKWARD);
    m_backLeft->run(FORWARD);
    m_backRight->run(BACKWARD); 

    m_frontLeft->setSpeed(190);
    m_frontRight->setSpeed(150);
    m_backLeft->setSpeed(190);
    m_backRight->setSpeed(150);   

    delay(m_rotTime);
    m_carRotation += 0.5*PI;
    goBrake(100, 0);
}

void MotionControl::setRotTime(int16_t time) {
    m_rotTime = time;
}

void MotionControl::goBrake(int delayMs, int reverseMs) {
    if (m_frontLeft->getSpeed() == 0) {
        return;
    }

    if (reverseMs > 0) {
        m_frontLeft->setSpeed(255);
        m_frontRight->setSpeed(255);
        m_backLeft->setSpeed(255);
        m_backRight->setSpeed(255);

        m_frontLeft->run((Direction)!m_driveDir);
        m_frontRight->run((Direction)!m_driveDir);
        m_backLeft->run((Direction)!m_driveDir);
        m_backRight->run((Direction)!m_driveDir);
        delay(reverseMs);
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

    Wire.begin();  // Initialize I2C
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);     // Wake up
    Wire.endTransmission(true);

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
    const float Kp_pos = 5.5;
    const float Kd_pos = 2.5;
    const float Kp_yaw = 5.8;
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
    
    if (trustLeft && trustRight) {
        // === POSITION ERROR ===
        if (!trustLeft) {
            leftCm = 15 - rightCm;
        }
        if (!trustRight) {
            rightCm = 15 - leftCm;
        }
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

    m_frontLeft->setSpeed(speedLeft);
    m_backLeft->setSpeed(speedLeft);
    m_frontRight->setSpeed(speedRight);
    m_backRight->setSpeed(speedRight);
}

void MotionControl::drive(double fromX, double fromY, int8_t toX, int8_t toY, double leftCm, double rightCm, double centerCm) {
    m_heading = calculateHeading(fromX, fromY, toX, toY);
    CompassDirMC newDir = radiansToDirection(m_heading-m_carRotation);
    
    if (newDir == CompassDirMC::East) {
        goRight();
        m_carRotation = m_heading;
        return;
    }

    if (newDir == CompassDirMC::West) {
        goLeft();
        m_carRotation = m_heading;
        return;
    }
    
    if (newDir == CompassDirMC::North)
        goForward();
    else
        goBackward();

    applyCorrection(leftCm, rightCm);
}

