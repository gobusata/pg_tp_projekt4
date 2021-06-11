#include "Robot.h"

Robot::Robot(REAL arm1_length_, REAL arm2_length_, REAL angle1_deg_, REAL angle2_deg_) :
	arm1_length(arm1_length_), arm2_length(arm2_length_), angle1(angle1_deg_), angle2(angle2_deg_),
	pen1(Color(100, 100, 100), arm_width), pen2(Color(70, 70, 70), arm_width),
	base_pos(30, 30)
{
	pen1.SetLineCap(LineCapRound, LineCapRound, DashCapRound);
	pen2.SetLineCap(LineCapRound, LineCapRoundAnchor, DashCapRound);

	set_arms(angle1, angle2);
	angle1 = angle1;
	angle2 = angle2;
	set_tAngles(angle1, angle2);

	stopped = false;
	tpos_reached = false;
}

void Robot::draw(Graphics* graphics)
{
	graphics->DrawLine(&pen1, base_pos, base_pos + arm1);
	graphics->DrawLine(&pen1, base_pos + PointF(0, 20), base_pos + arm1 + arm2*0.05);
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
	ang_vel1 += (tAngle1 - 2 * dzeta1 * omega1 * ang_vel1 - angle1) * dt;
	angle1 += omega1 * omega1 * ang_vel1 * dt;
	ang_vel2 += (tAngle2 - 2 * dzeta2 * omega2 * ang_vel2 - angle2) * dt;
	angle2 += omega2 * omega2 * ang_vel2 * dt;
	
	stopped = false;
	tpos_reached = false;

	float dang1 = abs(angle1 - tAngle1), dang2 = abs(angle2 - tAngle2);
	if ( dang1 * dang1 + abs(ang_vel1) * abs(ang_vel1) < 25 &&
		 dang2 * dang2 + abs(ang_vel2) * abs(ang_vel2) < 25)
	{
		this->stopped = true;
	}

	if (abs(angle1 - tAngle1) < 5 && abs(angle2 - tAngle2) < 5)
	{
		this->tpos_reached = true;
	}


	set_arms(angle1, angle2);
	
	if (this->following_trajectory)
	{
		this->follow_trajectory();
	}


	if (catched_triangle != nullptr)
	{
		catched_triangle->vel = (base_pos + arm1 + arm2 - catched_triangle->pos) * 10e-3;
	}
}

void Robot::set_arms(REAL angle1, REAL angle2)
{
	Gdiplus::Matrix m1, m2;
	m1.Rotate(angle1);
	m2.Rotate(angle2);

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
	angle1 = (a1 - a3) * 360 / 6.28;
	angle2 = (a1 + a2) * 360 / 6.28;
	angle2 -= angle1;

}

Vector2f Robot::calculateAngles(PointF p)
{
	float alpha1, alpha2;
	p = p - base_pos;
	REAL a1 = std::asin(p.Y / abs(p));
	REAL a2 = std::acos((-std::pow(arm1_length, 2) + std::pow(arm2_length, 2) + std::pow(abs(p), 2)) / (2 * abs(p) * arm2_length));
	REAL a3 = std::acos((std::pow(arm1_length, 2) - std::pow(arm2_length, 2) + std::pow(abs(p), 2)) / (2 * abs(p) * arm1_length));
	alpha1 = (a1 - a3) * 360 / 6.28;
	alpha2 = (a1 + a2) * 360 / 6.28;
	alpha2 -= alpha1;
	return Vector2f(alpha1, alpha2);
}

PointF Robot::get_positon()
{
	return base_pos + arm1 + arm2;
}

void Robot::set_tAngles(float a1, float a2)
{
	tAngle1 = a1;
	tAngle2 = a2;
}

void Robot::set_tPosition(PointF p)
{
	Vector2f v1 = this->calculateAngles(p);
	this->set_tAngles(v1[0], v1[1]);
}

void Robot::follow_trajectory()
{
	if (curr_tpos->robotCommand == rc_do_nothing)
	{
		if (this->tpos_reached)
		{
			curr_tpos++;
		}
	}
	else if (curr_tpos->robotCommand == rc_stop)
	{
		if (this->stopped)
		{
			curr_tpos++;
		}
	}
	else if (curr_tpos->robotCommand == rc_catch)
	{
		if (this->stopped)
		{
			curr_tpos++;
		}
	}

	if (curr_tpos != this->trajectory.cend())
	{
		this->set_tPosition(PointF(curr_tpos->pos.X, curr_tpos->pos.Y));
	}
	else
	{
		this->following_trajectory = false;
	}
}

void Robot::enter_trajectory(const std::vector<RobotPosition>& t)
{
	this->trajectory = t;
	this->curr_tpos = this->trajectory.cbegin();
	if (this->trajectory.size() != 0)
		this->set_tPosition(PointF(curr_tpos->pos.X, curr_tpos->pos.Y));
	this->following_trajectory = true;
}


