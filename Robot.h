#ifndef ROBOT_H
#define ROBOT_H
#include <Windows.h>
#include <gdiplus.h>
#include <cmath>

#include "Triangle.h"


using namespace Gdiplus;

inline REAL abs(const PointF& v)
{
	return std::sqrt(v.X * v.X + v.Y * v.Y);
}

class Robot
{
private:
	REAL arm_width = 5;
	Pen pen1, pen2;
	PointF base_pos, arm1, arm2;
	
public:
	Triangle* catched_triangle = nullptr;
	PointF catched_triangle_r;
	REAL arm1_length, arm2_length, angle1_deg, angle2_deg;

	Robot(REAL arm1_length_ = 300, REAL arm2_length_ = 300, REAL angle1_deg_ = -45, REAL angle2_deg_ = 45);

	void draw(Graphics* graphics);

	void catch_triangle(Triangle* tri);

	void update(REAL dt);

	void set_arms(REAL angle1, REAL angle2);

	void set_postion(PointF p);

	PointF get_positon();
};

#endif // !ROBOT_H
