///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Pranay Katyal
//////////////////////////////////////


#include "CollisionChecking.h"
#include <cmath>
#include <algorithm>

// Helper: check if point (x,y) is inside rectangle r.
bool pointInRectangle(double x, double y, const Rectangle &r)
{
    return (x >= r.x && x <= r.x + r.width &&
            y >= r.y && y <= r.y + r.height);
}

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto &r : obstacles)
    {
        if (pointInRectangle(x, y, r))
            return false;
    }
    return true;
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto &r : obstacles)
    {
        // Compute the closest point in the rectangle to the circle center
        double closestX = std::max(r.x, std::min(x, r.x + r.width));
        double closestY = std::max(r.y, std::min(y, r.y + r.height));
        double dx = x - closestX;
        double dy = y - closestY;
        if (dx * dx + dy * dy < radius * radius)
            return false;
    }
    return true;
}

// Helper: determine if any corner of a rotated square (centered at (x,y)) is inside rectangle r.
bool squareIntersectsRectangle(double x, double y, double theta, double sideLength, const Rectangle &r)
{
    double half = sideLength / 2.0;
    // Local coordinates for the four corners
    double corners[4][2] = {
        { -half, -half },
        {  half, -half },
        {  half,  half },
        { -half,  half }
    };
    for (int i = 0; i < 4; ++i)
    {
        // Rotate and translate the corner
        double rx = corners[i][0] * cos(theta) - corners[i][1] * sin(theta) + x;
        double ry = corners[i][0] * sin(theta) + corners[i][1] * cos(theta) + y;
        if (pointInRectangle(rx, ry, r))
            return true;
    }
    return false;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    for (const auto &r : obstacles)
    {
        if (squareIntersectsRectangle(x, y, theta, sideLength, r))
            return false;
    }
    return true;
}