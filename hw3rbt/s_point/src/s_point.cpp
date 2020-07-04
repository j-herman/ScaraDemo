#include "s_point/s_point.h"

SPoint::SPoint()
{
    x = y = z = 0;
}
SPoint::SPoint(float x_, float y_, float z_)
{
    x = x_;
    y = y_;
    z = z_;
}
SPoint::SPoint(geometry_msgs::Point p)
{
    x = p.x;
    y = p.y;
    z = p.z;
}
SPoint::~SPoint()
{
    //dtor
}

float SPoint::point_compare(SPoint p)
{
    return sqrt(pow(x-p.Getx(),2)+pow(y-p.Gety(),2)+pow(z-p.Getz(),2));
}

std::ostream& operator<<(std::ostream& os, const SPoint& p)
{
    os<<std::setprecision(4)<<"x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z;
    return os;
}
