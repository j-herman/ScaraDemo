#ifndef SPOINT_H
#define SPOINT_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include "geometry_msgs/Point.h"

class SPoint
{
    public:
        SPoint();
        SPoint(float x_, float y_, float z_);
        SPoint(geometry_msgs::Point p);
        ~SPoint();

        float Getx() { return x; }
        void Setx(float val) { x = val; }
        float Gety() { return y; }
        void Sety(float val) { y = val; }
        float Getz() { return z; }
        void Setz(float val) { z = val; }
        friend std::ostream& operator<<(std::ostream& os, const SPoint& p);
        float point_compare(SPoint p);

    private:
        float x;
        float y;
        float z;
};

#endif // SPOINT_H
