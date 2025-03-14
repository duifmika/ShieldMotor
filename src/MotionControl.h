#include "ShieldMotor.h"

class MotionControl {
public:
    MotionControl() = default;

    void init(double cellWidth, double wallWidth, 
            ShieldMotor* frontLeft, ShieldMotor* frontRight, 
            ShieldMotor* backLeft, ShieldMotor* backRight);
private:
    double m_heading = 0.f; // direction of motion in radians where 0 is North (top of the maze)
    double m_carRotation = 0.f; // car rotation

    double m_wallWidth;
    double m_cellWidth; // assuming square cells

    ShieldMotor* m_frontLeft; 
    ShieldMotor* m_frontRight;
    ShieldMotor* m_backLeft;
    ShieldMotor* m_backRight;
public:
    double getHeading() const;
    double getCarRotation() const;

    bool rotateTo(double heading); // return true if fully rotated
    bool drive(double fromX, double fromY, int8_t toX, int8_t toY); // return true when at "to" position.
private:
    double calculateHeading(double fromX, double fromY, int8_t toX, int8_t toY) const; // calculate heading to next position (new car heading)
    double calculateDistance(double fromX, double fromY, int8_t toX, int8_t toY) const;

};
