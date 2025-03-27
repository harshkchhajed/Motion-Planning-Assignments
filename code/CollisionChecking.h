#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>

struct Rectangle
{
    // Lower left corner and dimensions
    double x, y;
    double width, height;
};

// Check if the point (x, y) is free (i.e. not inside any obstacle)
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Check if a circle with center (x,y) and given radius is free from obstacles
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

// Check if a square with center (x,y), orientation theta, and side length is free from obstacles
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

#endif