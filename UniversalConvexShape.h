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
#include "ProjektLogs.h"

using namespace Eigen;
template <typename Type>
using Vector2 = Eigen::Matrix<Type, 2, 1>;

struct GjkSimplex;
struct VectorWithIndex;

class ProjektConvexPolygon
{
	Affine2f tm;
	Vector3f pos;
	std::vector<Vector2f> gs;
public:
	// vertices of the bounding polygon
	std::vector<Vector2f> vs;
	ProjektConvexPolygon() : vs() {};
	ProjektConvexPolygon(const std::vector<Vector2f>& _vs);
	void setTranslation(const Vector2f& _t);
	void setRotation(float _r);
	void setPosition(const Vector3f& _pos);
	bool addVertex(int i, Vector2f v);
	bool addVertex(VectorWithIndex v);
	Vector2f getPosition() const;
	Vector2f getVertex(int i) const;
	Vector2f getTransformedVertex(int i) const;
	Vector2f getVertexPos(int i) const;
	Vector2f getVertexPos(const Vector2f& v) const;
	void offsetVertices(Vector2f _v);
	VectorWithIndex gjkSupportVer(const Vector2f& _dir) const;
	friend GjkSimplex & gjkSimplex(const ProjektConvexPolygon& a, const ProjektConvexPolygon& b, GjkSimplex & is);
private:
	bool dir_lt(const Vector2f& a, const Vector2f& b) const;
	bool dir_gt(const Vector2f& a, const Vector2f& b) const;
};

GjkSimplex & gjkSimplex(const ProjektConvexPolygon& a, const ProjektConvexPolygon& b, GjkSimplex & is);

class ProjektConvexBody
{
public:
	ProjektConvexPolygon pcp;
	Vector3f pos, vel;
	DiagonalMatrix<float, 3> kmat;
	float mass, inertia, sphere_bound;
	static float gravity;
	Gdiplus::GraphicsPath shape;
	void createShape();
	ProjektConvexBody();
	ProjektConvexBody(const ProjektConvexBody&);
	ProjektConvexBody(const Vector3f& _pos, const Vector3f& _vel, const std::vector<Vector2f>& _vs, float _mass, float _inertia);
	void draw(Gdiplus::Graphics* lpgraphics, Gdiplus::Pen* lppen, Gdiplus::Brush* lpbrush);
	void updateVel(float dt);
	void updatePos(float dt);
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

struct GjkSimplex
{
	enum SimpStateEnum
	{
		uninitialized, point, line, triangle
	};
	SimpStateEnum simpstate = uninitialized;
	bool collision;
	std::array<VectorWithIndex, 6> simplex_vertices;
	std::array<Vector2f, 3> simplex;
	void refresh(const ProjektConvexPolygon & _a, const ProjektConvexPolygon & _b);
};

template<>
struct fmt::formatter<GjkSimplex>
{
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        auto it = ctx.begin(), end = ctx.end();
        return end;
    }

    template <typename FormatContext>
    auto format(const GjkSimplex& gs, FormatContext& ctx) -> decltype(ctx.out())
    {
        //*ctx.out() = '\n';
        switch (gs.simpstate)
        {
        case GjkSimplex::point:
            format_to(ctx.out(), "simpstate: vertex");
            break;
        case GjkSimplex::line:
            format_to(ctx.out(), "simpstate: line\n");
			format_to(ctx.out(), "simplex: {:.2f}, {:.2f}", gs.simplex[0], gs.simplex[1]);
            break;
        case GjkSimplex::triangle:
            format_to(ctx.out(), "simpstate: triangle");
			format_to(ctx.out(), "simplex: {:.2f}, {:.2f}, {:.2f}", gs.simplex[0], gs.simplex[1], gs.simplex[2]);
            break;
        }
		return ctx.out();
    }
};

Gdiplus::GraphicsPath* createGraphicsPath(const std::vector<Vector2f>& _vs);

/// <summary>
/// checks using sheres bounding ProjektConvexBody instances, wether shapes are in close proximity
/// </summary>
bool collisionDetectionWide(const ProjektConvexBody&, const ProjektConvexBody&);

Vector2f findCenterOfMass(const std::vector<Vector2f>& _vs);

float findMomentOfInertia(const std::vector<Vector2f>& _vs);

GjkSimplex collisionDetection(const ProjektConvexBody& a, const ProjektConvexBody& b);

/// <summary>
/// calculates shortest distance between triangle and point 
/// </summary>
/// <param name="tri">list of triangles vertices' coordinates</param>
/// <param name="p">point coordinates</param>
/// <returns></returns>
ClosestFeature pointToTriangle(std::array<Vector2f, 3> tri, const Vector2f& p);

ClosestFeature pointToLine(std::array<Vector2f, 2> line, const Vector2f& p);

Vector2f barycentricCoordinates2(std::array<Vector2f, 2> line, const Vector2f);

Vector3f barycentricCoordinates3(std::array<Vector2f, 3> tri, const Vector2f);

/// <summary>
/// computes cartrsian coordinates of a point from barycentric coordinates
/// </summary>
/// <param name="tri">vertices of triangle, for which barycentric coordinates have been computed</param>
/// <param name="bc">barycnetric coordinates</param>
/// <returns></returns>
Vector2f cartesianCoordinates(const std::array<Vector2f, 3>& tri, const Vector3f& bc);

Vector2f cartesianCoordinates(const std::array<Vector2f, 2>& line, const Vector2f& bc);

float cross(const Vector2f&, const Vector2f&);

Vector2f cross(const Vector2f& v1, float v2);

Vector2f cross(float v1, const Vector2f& v2);

template <typename T> int sgn(T val);

inline Gdiplus::PointF ev2gp(const Vector2f& v) { return Gdiplus::PointF(v.x(), v.y()); }

#endif
