#ifndef VEC2_H
#define VEC2_H

#include <iostream>
#include <stdexcept>

/*
 * Coordinate for planar space; in Webots it corresponds to X & Z coordinates
 */
struct Vec2
{
    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(double argX, double argY) : x(argX), y(argY) {}
    
    Vec2& operator=(const Vec2& other)
    {
        x = other.x;
        y = other.y;
        return *this;
    }
    
    const double &operator[](unsigned int index) const
    {
        if (index == 0)
            return x;
        if (index == 1)
            return y;
        std::cout << "ERROR: Wrong vector index: " << index << std::endl;
        throw std::out_of_range("Invalid index");
    }
    
    double &operator[](unsigned int index)
    {
        if (index == 0)
            return x;
        if (index == 1)
            return y;
        std::cout << "ERROR: Wrong vector index: " << index << std::endl;
        throw std::out_of_range("Invalid index");
    }
    
    Vec2& operator*=(const double s)
    {
        x *= s;
        y *= s;
        return *this;
    }
    
    Vec2& operator/=(const double s)
    {
        x /= s;
        y /= s;
        return *this;
    }
    
    Vec2& operator-=(const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }
    
    Vec2& operator+=(const Vec2& v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }
    
    Vec2 operator-(const Vec2& other) const
    {
        return Vec2(x - other.x,
                    y - other.y);
    }
    
    Vec2 operator/(double factor) const
    {
        return Vec2(x/factor,
                    y/factor);
    }
    
    Vec2 operator*(double factor) const
    {
        return Vec2(x*factor,
                    y*factor);
    }
    
    double x;
    double y;
};

double norm(const Vec2 &v)
{
  return std::sqrt(v[0]*v[0] + v[1]*v[1]);
}

std::ostream& operator<<(std::ostream& out, Vec2 const& p)
{
    return out << "{ " << p.x << ", " << p.y << " }";
}

#endif
