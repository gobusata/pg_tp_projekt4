#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <Windows.h>
#include <gdiplus.h>
#include <cmath>
#include <algorithm>
#include <gsl/gsl_linalg.h>
#include <Eigen/dense>
#include <Eigen/geometry>

using namespace Eigen;
using namespace Gdiplus;

PointF operator*(PointF p, REAL f);
inline REAL my_round(REAL a, REAL precission);
inline PointF my_round(const PointF& p, REAL precission);
inline REAL inner_product(const PointF& p1, const PointF& p2);

class Triangle
{

public:
	PointF pos, ver[3], vel{ 0.0001, 0 };
	REAL size = 50, mass = 1, inertia = mass*size*size/12, omega = 0.0001, bounding_sphere = 1.1*size;
	static PointF gravity;
	Triangle(PointF pos_, REAL size_, REAL initial_rotation_ = 120, PointF vel_ = { 0.0001, 0 }, REAL omega_ = 0.0002);

	void draw(Graphics* lpgraphics_, Pen* lppen_);

	void update(REAL dt);

	void update(REAL dt, PointF* relative_vector);

	void collision_with_wall(const RectF* walls);
	
	void collision_with_figure(Triangle& tri, REAL dt);
	
	void collision_with_figure2(PointF& vel1, REAL& omega1, const PointF& r1, PointF& vel2, REAL& omega2, const PointF& r2, const Triangle& tri, const PointF& sur);

	void collision_with_static_figure(PointF& vel, REAL& omega, const PointF& r);
	
	static BOOL intersect_segments(IN PointF a1, IN PointF b1, IN PointF a2, IN PointF b2, OUT PointF  res[2]);

	static bool is_point_int_triangle(const Triangle* tri, const PointF& point);
};


inline PointF operator*(PointF p, REAL f)
{
	return PointF(p.X * f, p.Y * f);
}

inline REAL my_round(const REAL a, REAL precission)
{
	if (abs(a) < precission)
	{
		return 0;
	}
	else
	{
		return a;
	}
}

inline PointF my_round(const PointF & p, REAL precission)
{
	return PointF(my_round(p.X, precission), my_round(p.Y, precission));
}




#endif

