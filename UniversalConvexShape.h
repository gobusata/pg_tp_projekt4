#ifndef UNIVERSAL_CONVEX_SHAPE_H
#define UNIVERSAL_CONVEX_SHAPE_H

#include <windows.h>
#include <gdiplus.h>
#include <gdipluspath.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/dense>
#include <Eigen/geometry>
#include <gsl/gsl_linalg.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#undef max
#include "ProjektLogs.h"

using namespace Gdiplus;
using namespace Eigen;

struct Intersection;
struct VectorWithIndex;
class UniversalConvexShape
{
public:
	Vector3f pos, vel;
	std::vector <Vector2f> vertices;
	float mass, inertia, sphere_bound;
	static float gravity;
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

};

struct VectorWithIndex : public Vector2f
{
public:
	int index;
	
	VectorWithIndex() = default;

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
			VectorWithIndex point;
			Vector2f normal;
		};
		struct {
			float distance;
			std::vector<VectorWithIndex> aClosestFeature, bClosestFeature;
		};
	};
	
	Intersection() : collision{ false }, distance(0), aClosestFeature{}, bClosestFeature{} {};
	
	Intersection(bool a);
	
	Intersection(bool a, VectorWithIndex b, Vector2f c) : collision(true), point(b), normal(c) {};
	
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

ClosestFeature pointToLine(std::vector<Vector2f> line, const Vector2f& p);

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
