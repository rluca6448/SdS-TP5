#pragma once

#include "types/Vector2D.h"

namespace types {

enum class MotionScheme { TimeDriven, EventDriven };

class Particle {
public:
    Particle(int id, Vector2D position, double radius)
        : id(id), position(position), radius(radius) {}

    int getId() const { return id; }
    Vector2D getPosition() const { return position; }
    void setPosition(Vector2D value) { position = value; }
    Vector2D getVelocity() const { return velocity; }
    void setVelocity(Vector2D value) { velocity = value; }
    Vector2D getTarget() const { return target; }
    void setTarget(Vector2D value) { target = value; }
    double getRadius() const { return radius; }
    void setRadius(double value) { radius = value; }
    MotionScheme getScheme() const { return scheme; }
    void setScheme(MotionScheme value) { scheme = value; }

private:
    int id;
    Vector2D position;
    Vector2D velocity;
    Vector2D target;
    double radius;
    MotionScheme scheme = MotionScheme::TimeDriven;
};

}
