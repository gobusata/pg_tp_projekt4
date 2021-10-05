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
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

using namespace Gdiplus;
using namespace Eigen;

struct Intersection;
struct VectorWithIndex;
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

	Vector2f getTransformedVertex(int i);

	Vector2f getVertexPos(int i);
	
	void draw(Graphics* lpgraphics, Pen* lppen, Brush* lpbrush);
	
	void updateVel(float dt);
	
	void updatePos(float dt);
	
	VectorWithIndex gjkSupportVer(const Vector2f&) const;
	
	void collisionWithRect(const Gdiplus::RectF& rect, float);
	
	Intersection collisionWithPlane(Vector2f position, Vector2f dir);
	
	/// <summary>
	/// Calculates change of velocity of miving object coliding with fixed obstacle.
	/// Uses mass and inertia of UniversaConvexShape instance it belogns to but does not change its velocity and ang. vel..
	/// </summary>
	/// <param name="vel">velocity of moving object</param>
	/// <param name="omega">angular velocity of moving object</param>
	/// <param name="r">vector from point of rotation of the moving object to the point of contact</param>
	/// <param name="p">unused</param>
	void collisionWithFixedObject(Vector2f& vel, float& omega, const Vector2f& r, const Vector2f& p);
	
	void collisionWithHorizontalPlane(Vector2f& vel, float& omega, const Vector2f& r, const Vector2f& p);
};

struct VectorWithIndex : public Vector2f
{
public:
	int index;
	
	VectorWithIndex() = default;

	VectorWithIndex& operator=(const VectorWithIndex& a) = default;

	VectorWithIndex(const VectorWithIndex& a) : Vector2f{ a }, index{ a.index } {};

	VectorWithIndex(int a, Vector2f b) : index{ a }, Vector2f{ b } {};
	
	VectorWithIndex(Vector2f a, int b) : index{ b }, Vector2f{ a } {};
};

struct ClosestFeature
{
	std::vector<VectorWithIndex>feature;
	
	float distance;
	
	ClosestFeature() = default;
	
	ClosestFeature(std::vector<VectorWithIndex> a, float b) : feature{ a }, distance{ b } {};
};
/// <summary>
/// Class describing contact of to convex 2D figures
/// </summary>
struct Intersection
{
	bool collision;
	union {
		struct {
			Vector2f point, normal;
		};
		struct {
			float distance;
			std::vector<VectorWithIndex> aClosestFeature, bClosestFeature;
		};
	};
	
	ClosestFeature closestFeature;

	Intersection() : collision{ false }, distance(0), aClosestFeature{}, bClosestFeature{} {};
	
	Intersection(bool a);
	
	Intersection(bool a, Vector2f b, Vector2f c) : collision(true), point(b), normal(c) {};
	
	Intersection(bool a, float b, std::vector<VectorWithIndex> c, std::vector<VectorWithIndex> d) :
		collision{ false }, distance{ b }, aClosestFeature{ c }, bClosestFeature{ d } {};
	
	Intersection(const Intersection& a);
	
	Intersection& operator=(const Intersection& a);
	
	~Intersection();
	

};


Intersection gjkSimplex(const UniversalConvexShape&, const UniversalConvexShape&);

/// <summary>
/// checks using sheres bounding UniversalConvexShape instances, wether shapes are in close proximity
/// </summary>
bool collisionDetectionWide(const UniversalConvexShape&, const UniversalConvexShape&);

Intersection collisionDetection(const UniversalConvexShape& a, const UniversalConvexShape& b);

//void collisionDetectionNarrow(UniversalConvexShape& a, const UniversalConvexShape& b);

void collisionWithMovingObjectSol(float, float, Vector2f&, float&, Vector2f,
	float, float, Vector2f&, float&, Vector2f);

void collisionWithMovingObject(UniversalConvexShape&, UniversalConvexShape&);

/// <summary>
/// calculates shortest distance between triangle and point 
/// </summary>
/// <param name="tri">list of triangles vertices' coordinates</param>
/// <param name="p">point coordinates</param>
/// <returns></returns>
ClosestFeature pointToTriangle(std::vector<Vector2f> tri, const Vector2f& p);

Vector2f barycentricCoordinates2(std::vector<Vector2f> line, const Vector2f);

Vector3f barycentricCoordinates3(std::vector<Vector2f> tri, const Vector2f);

/// <summary>
/// computes cartrsian coordinates of a point from barycentric coordinates
/// </summary>
/// <param name="tri">vertices of triangle, for which barycentric coordinates have been computed</param>
/// <param name="bc">barycnetric coordinates</param>
/// <returns></returns>
Vector2f cartesianCoordinates(const std::vector<Vector2f> tri, const Vector3f bc);

float cross(const Vector2f&, const Vector2f&);

Vector2f cross(const Vector2f& v1, float v2);

Vector2f cross(float v1, const Vector2f& v2);

template <typename T> int sgn(T val);

#endif
