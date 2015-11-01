/*
 * definitions.h
 *
 *  Created on: Jun 16, 2014
 *      Author: aida
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <cmath>

namespace aida {

struct Point {
	float x, y, z;

	Point(float x_, float y_, float z_ = 0) : x(x_), y(y_), z(z_) {}
	Point() : x(0), y(0), z(0) {}
};

struct Line {
	float slope, y;
	bool isVertical;

	Line(float slope_, float y_) : slope(slope_), y(y_), isVertical(false) {}
	Line() : slope(0), y(0), isVertical(false) {}

	float getY(float x)
	{
		return x * slope + y;
	}
};

struct Plane {
	float a, b, c, d;

	Plane(float a_, float b_, float c_, float d_): a(a_), b(b_), c(c_), d(d_) {}
	Plane(): a(0), b(0), c(0), d(0) {}

	float getY(float x, float z)
	{
		return -(a*x + c*z + d)/b;
	}
};

float abs(float x)
{
	return x < 0 ? -x : x;
}

float square (float x)
{
	return x*x;
}

float dis(Point a, Point b)
{
	return sqrt(square(a.x-b.x) + square(a.y-b.y) + square(a.z-b.z));
}

float dis(Point v, Plane p)
{
	return abs((p.a*v.x + p.b*v.y + p.c*v.z + p.d) / sqrt(square(p.a) + square(p.b) + square(p.c)));
}

float dis(Line a, Line b)
{
	return abs(a.y - b.y)/sqrt(a.slope * b.slope + 1);	//assuming a and b are parallel
}

struct Time {
	int sec, nsec;
};

bool equ(const char* s, const char* t)
{
	for (int i = 0; s[i] != 0 || t[i] != 0; i++)
		if (s[i] != t[i])
			return false;
	return true;
}

}

#endif /* DEFINITIONS_H_ */
