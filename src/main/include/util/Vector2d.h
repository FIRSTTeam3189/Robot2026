#pragma once

#define _USE_MATH_DEFINES
#include <cmath> 
#include <frc/geometry/Pose2d.h>


struct Vector2d {

    double x; 
    double y; 

    Vector2d() : x(0), y(0) {} 
    Vector2d(double x, double y) : x(x), y(y) {}
    // Angle in degrees 
    Vector2d(double angle) {
        double radians = ToRadians(angle); 
        x = std::cos(radians); 
        y = std::sin(radians);  
    }

    Vector2d(frc::Pose2d pose) {
        x = pose.X().value();
        y = pose.Y().value(); 
    }   

    Vector2d(frc::Pose2d from, frc::Pose2d to) {
        x = to.X().value() - from.X().value();
        y = to.Y().value() - from.Y().value(); 
    }

    double Length() const {
        return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); 
    }

    Vector2d Normalize() {
        double length = this->Length(); 
        if (length == 0) return Vector2d(0, 0); 
        return Vector2d(x / length, y / length); 
    }

    Vector2d operator+(const Vector2d& other) const {
        return Vector2d(x + other.x, y + other.y); 
    }

    Vector2d operator+=(const Vector2d& other) const {
        return Vector2d(x + other.x, y + other.y); 
    }

    Vector2d operator-(const Vector2d& other) const {
        return Vector2d(x - other.x, y - other.y); 
    }

    Vector2d operator-=(const Vector2d& other) const {
        return Vector2d(x - other.x, y - other.y); 
    }

    Vector2d operator*(double scale) const {
        return Vector2d(x * scale, y * scale); 
    }
   
    Vector2d operator/(double scale) const {
        if (scale == 0) return Vector2d(0, 0);
        return Vector2d(x / scale, y / scale); 
    } 

    // Angle in radians 
    Vector2d Rotate(double angle) const {
        return Vector2d(x * std::cos(angle) - y * std::sin(angle),
        x * std::sin(angle) + y * std::cos(angle)); 
    }

    double Dot(const Vector2d& other) const {
        return x * other.x + y * other.y;
    }

    double Cross(const Vector2d& other) const {
        return x * other.y - y * other.x; 
    }

    double Angle() const {
        return std::atan(y / x); 
    }

    // Angle from 0-pi radians 
    double AngleBetween(const Vector2d& other) const {
        double radians = std::acos(Dot(other) / (Length() * other.Length())); 
        return ToAngle(radians);
    }
    
    // Angle from -pi to pi 
    double SignedAngle(const Vector2d& other) const {
        double radians = std::atan2(Cross(other), Dot(other)); 
        return ToAngle(radians); 
    }

    static double ToRadians(double angle) {
        return angle * M_PI / 180; 
    } 
    static double ToAngle(double radians) {
        return radians * 180 / M_PI; 
    }

};