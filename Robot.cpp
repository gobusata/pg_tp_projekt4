#include "Robot.h"

Robot::Robot(REAL arm1_length_, REAL arm2_length_, REAL angle1_deg_, REAL angle2_deg_) :
	arm1_length(arm1_length_), arm2_length(arm2_length_), angle1_deg(angle1_deg_), angle2_deg(angle2_deg_),
	pen1(Color(100, 100, 100), arm_width), pen2(Color(70, 70, 70), arm_width),
	base_pos(30, 30)
{
	pen1.SetLineCap(LineCapRound, LineCapRound, DashCapRound);
	pen2.SetLineCap(LineCapRound, LineCapRoundAnchor, DashCapRound);

	set_arms(angle1_deg, angle2_deg);

}

void Robot::draw(Graphics* graphics)
{
	graphics->DrawLine(&pen1, base_pos, base_pos + arm1);
	graphics->DrawLine(&pen2, base_pos + arm1, base_pos + arm1 + arm2);
}

void Robot::catch_triangle(Triangle* tri)
{

	if (catched_triangle == nullptr)
	{
		if (Triangle::is_point_int_triangle(tri, base_pos + arm1 + arm2))
		{
			catched_triangle = tri;
			catched_triangle_r = base_pos + arm1 + arm2 - tri->pos;
		}
		else
		{
			catched_triangle = nullptr;
		}
	}

}

void Robot::update(REAL dt)
{
	if (catched_triangle != nullptr)
	{
		catched_triangle->vel = (base_pos + arm1 + arm2 - catched_triangle->pos) * 10e-3;

		//catched_triangle_r = base_pos + arm1 + arm2 - catched_triangle->pos;
		//catched_triangle->collision_with_static_figure(catched_triangle->vel, catched_triangle->omega, catched_triangle_r);
	}
}

void Robot::set_arms(REAL angle1, REAL angle2)
{
	angle1_deg = angle1;
	angle2_deg = angle2;
	Matrix m1, m2;
	m1.Rotate(angle1_deg);
	m2.Rotate(angle2_deg);

	PointF vec1(arm1_length, 0), vec2(arm2_length, 0);
	arm1 = { arm1_length, 0 };
	arm2 = { arm2_length, 0 };
	m1.TransformVectors(&arm1);
	m1.TransformVectors(&arm2);
	m2.TransformVectors(&arm2);
}

void Robot::set_postion(PointF p)
{
	p = p - base_pos;
	REAL a1 = std::asin(p.Y / abs(p));
	REAL a2 = std::acos((-std::pow(arm1_length, 2) + std::pow(arm2_length, 2) + std::pow(abs(p), 2)) / (2 * abs(p) * arm2_length));
	REAL a3 = std::acos((std::pow(arm1_length, 2) - std::pow(arm2_length, 2) + std::pow(abs(p), 2)) / (2 * abs(p) * arm1_length));
	angle1_deg = (a1 - a3) * 360 / 6.28;
	angle2_deg = (a1 + a2) * 360 / 6.28;
	angle2_deg -= angle1_deg;
	set_arms(angle1_deg, angle2_deg);

}

PointF Robot::get_positon()
{
	return base_pos + arm1 + arm2;
}
