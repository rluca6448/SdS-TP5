#pragma once

#include <cmath>

namespace types {

struct Vector2D {
    double x = 0.0;
    double y = 0.0;

    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }

    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    double dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    double norm() const { return std::hypot(x, y); }

    Vector2D normalized() const {
        const double length = norm();
        return length > 0 ? *this * (1.0 / length) : Vector2D{};
    }
};

}
