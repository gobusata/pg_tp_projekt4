#ifndef UNIVERSAL_CONVEX_SHAPE_H
#define UNIVERSAL_CONVEX_SHAPE_H

#include <windows.h>
#include <gdiplus.h>
#include <gdipluspath.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <vector>
#include <algorithm>
#include <Eigen/dense>
#include <Eigen/geometry>
#include <gsl/gsl_linalg.h>

using namespace Gdiplus;
using namespace Eigen;

struct Intersection;
class UniversalConvexShape
{
public:
	Vector2f pos, vel;
	std::vector <Vector2f> vertices;
	float mass, inertia, omega, rot, sphere_bound;
	static Vector2f gravity;
	GraphicsPath shape;

	void createShape();
public:
	UniversalConvexShape();
	UniversalConvexShape(const UniversalConvexShape&);
	void draw(Graphics* lpgraphics, Pen* lppen, Brush* lpbrush);
	void update(float);
	Vector2f gjkSupportVer(const Vector2f&) const;
	void collisionWithRect(const Gdiplus::RectF& rect, float);
	Intersection collisionWithPlane(Vector2f, Vector2f);
	void collisionWithFixedObject(Vector2f&, float&, const Vector2f&, const Vector2f&);
	
	
	


};

struct Intersection
{
	bool collision;
	Vector2f point;
	Intersection(bool a) : collision(a), point{ 0, 0 } {};
	Intersection(bool a, Vector2f b) : collision(a), point(b) {};
};

Intersection gjkSimplex(const UniversalConvexShape&, const UniversalConvexShape&);
Intersection collisionDetection(const UniversalConvexShape&, const UniversalConvexShape&);
void collisionWithMovingObjectSol(float, float, Vector2f&, float&, Vector2f,
	float, float, Vector2f&, float&, Vector2f);
void collisionWithMovingObject(UniversalConvexShape&, UniversalConvexShape&);

Vector2f barycentricCoordinates2(const Vector2f[2], const Vector2f);
Vector3f barycentricCoordinates3(const Vector2f[3], const Vector2f);
float cross(const Vector2f&, const Vector2f&);
std::vector<Vector2f> pointToTriangle(const Vector2f[3], const Vector2f& );




#endif
