#include "ShieldMotor.h"

enum class CompassDirMC : int8_t {
    North = (1 << 0),
    South = (1 << 1),
    East = (1 << 2),
    West = (1 << 3)
};

#define BRAKE_TIME 0 

class MotionControl {
public:
    MotionControl() = default;

    void init(double cellWidth, double wallWidth, 
            ShieldMotor* frontLeft, ShieldMotor* frontRight, 
            ShieldMotor* backLeft, ShieldMotor* backRight);
private:
    double m_heading = 0.f; // direction of motion in radians where 0 is North (top of the maze)
    double m_carRotation = 0.f; // car rotation
    int16_t m_rotTime = 420;

    double m_wallWidth;
    double m_cellWidth; // assuming square cells

    ShieldMotor* m_frontLeft; 
    ShieldMotor* m_frontRight;
    ShieldMotor* m_backLeft;
    ShieldMotor* m_backRight;

public:
    Direction m_driveDir = FORWARD;

    double getHeading() const;
    double getCarRotation() const;
    void setRotTime(int16_t time);

    void drive(double fromX, double fromY, int8_t toX, int8_t toY, double leftCm, double rightCm, double centerCm); 
    void applyCorrection(double leftCm, double rightCm);
    void goForward();
    void goBackward();
    void goLeft();
    void goRight();
    void goBrake(int delayMs = BRAKE_TIME);

    double calculateDistance(double fromX, double fromY, int8_t toX, int8_t toY) const;
private:
    CompassDirMC radiansToDirection(double angleRad) const;
    double calculateHeading(double fromX, double fromY, int8_t toX, int8_t toY) const; 

};
