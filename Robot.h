#ifndef ROBOT_H
#define ROBOT_H
#include <Windows.h>
#include <gdiplus.h>
#include <cmath>
#include <Eigen/dense>
#include <vector>
#include <concepts>
#include <type_traits>



using namespace Gdiplus;
using namespace Eigen;

inline REAL abs(const PointF& v)
{
	return std::sqrt(v.X * v.X + v.Y * v.Y);
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, Gdiplus::PointF>::type operator*(const T a, const Gdiplus::PointF& b) 
{
	return Gdiplus::PointF(a * b.X, a * b.Y);
}

template <typename T> 
std::enable_if_t<std::is_floating_point<T>::value, Gdiplus::PointF> operator*(const Gdiplus::PointF& a, const T b) 
{
	return Gdiplus::PointF(b * a.X, b * a.Y); 
}


enum RobotCommand
{
	rc_stop, rc_release, rc_catch, rc_do_nothing
};

struct RobotPosition
{
	Point pos;
	RobotCommand robotCommand;
};

class Robot
{
public:
	REAL arm_width = 5;
	Pen pen1, pen2;
	PointF base_pos, arm1, arm2;
	float angle1, angle2, tAngle1, tAngle2;
	float ang_vel1 = 0, ang_vel2 = 0;
	float omega1 = 0.005, omega2 = 0.002;
	float dzeta1 = 0.6, dzeta2 = 0.6;
	bool stopped, tpos_reached;
	bool following_trajectory;
	std::vector<RobotPosition> trajectory;
	std::vector<RobotPosition>::const_iterator curr_tpos;

	void follow_trajectory();

public:
	PointF catched_triangle_r;
	REAL arm1_length, arm2_length;

	Robot(REAL arm1_length_ = 300, REAL arm2_length_ = 300, REAL angle1_deg_ = 90, REAL angle2_deg_ = 0);

	void draw(Graphics* graphics);

	void update(REAL dt);

	void set_arms(REAL angle1, REAL angle2);

	void set_tAngles(float a1, float a2);

	void set_tAngle1(float a) { tAngle1 = a; }

	void set_tAngle2(float a) { tAngle2 = a; }

	void set_postion(PointF p);

	void set_tPosition(PointF);

	Vector2f calculateAngles(PointF);

	void enter_trajectory(const std::vector<RobotPosition>&);

	PointF get_positon();
};



#endif // !ROBOT_H
