#include "UniversalConvexShape.h"

float UniversalConvexShape::gravity = 1e-5;

float cross(const Vector2f& v1, const Vector2f& v2)
{
	return v1[0] * v2[1] - v1[1] * v2[0];
}

Vector2f cross(const Vector2f& v1, float v2)
{
	return Vector2f(v1[1] * v2, -v1[0] * v2);
}

Vector2f cross(float v1, const Vector2f& v2)
{
	return Vector2f(-v1 * v2[1], v1 * v2[0]);
}

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template int sgn(float val);
template int sgn(double val);

UniversalConvexShape::UniversalConvexShape() :
	pos(200, 200, 0), vel(0.f, 0.f, 0.f), vertices(), mass(1), inertia(100),
	shape(FillModeWinding), sphere_bound(10) {}

UniversalConvexShape::UniversalConvexShape(const UniversalConvexShape& ucs) :
	pos(ucs.pos), vel(ucs.vel), vertices(ucs.vertices), mass(ucs.mass),
		inertia(ucs.inertia), shape(ucs.shape.GetFillMode()), sphere_bound(ucs.sphere_bound)

{
	PointF* points = new PointF[ucs.shape.GetPointCount()]();
	BYTE* pointTypes = new BYTE[ucs.shape.GetPointCount()]();
	ucs.shape.GetPathPoints(points, ucs.shape.GetPointCount());
	ucs.shape.GetPathTypes(pointTypes, ucs.shape.GetPointCount());
	GraphicsPath tmp(points, pointTypes, ucs.shape.GetPointCount(), ucs.shape.GetFillMode());
	shape.AddPath(&tmp, true);
	delete[]points;
	delete[]pointTypes;
}

Vector2f UniversalConvexShape::getTransformedVertex(int i)
{
	Affine2f trans1{};
	trans1.setIdentity();
	return trans1.rotate(pos(2)).linear() * vertices[i];
}

Vector2f UniversalConvexShape::getVertexPos(int i)
{
return pos.head<2>() + getTransformedVertex(i);
}

void UniversalConvexShape::draw(Graphics* lpgraphics, Pen* lppen, Brush* lpbrush)
{
	Gdiplus::Matrix matrix;
	matrix.Translate(pos[0], pos[1]);
	matrix.Rotate(double(this->pos(2)*180.0/3.14), MatrixOrderPrepend);
	lpgraphics->SetTransform(&matrix);
	if(lpbrush != nullptr)
		lpgraphics->FillPath(lpbrush, &shape);
	if(lppen != nullptr)
		lpgraphics->DrawPath(lppen, &shape);
	lpgraphics->ResetTransform();
}

void UniversalConvexShape::createShape()
{
	for (int i = 1; i < vertices.size(); i++)
	{
		Vector2f v1{ vertices[i - 1] };
		Vector2f v2{ vertices[i] };
		v1 += 0 * v1.normalized();
		v2 += 0 * v2.normalized();

		shape.AddLine(v1[0], v1[1], v2[0], v2[1]);
	}

	Vector2f v1{ *(vertices.end() - 1) }, v2{ vertices[0] };
	v1 += 0.0 * v1.normalized();
	v2 += 0.0 * v2.normalized();

	shape.AddLine(v1[0], v1[1], v2[0], v2[1]);

	this->sphere_bound = std::max_element(vertices.begin(), vertices.end(), \
		[](Vector2f a, Vector2f b)->bool {return a.squaredNorm() < b.squaredNorm(); })->norm()*1.1;
}

void UniversalConvexShape::updateVel(float dt)
{
	vel = vel + Vector3f( 0.f, gravity, 0.f ) *dt;
}

void UniversalConvexShape::updatePos(float dt)
{
	pos = pos + vel * dt;
}

VectorWithIndex UniversalConvexShape::gjkSupportVer(const Vector2f& dir_) const 
{
	Vector2f dir{ Rotation2Df(-pos(2)) * dir_ };
	int index = std::max_element(vertices.begin(), vertices.end(), 
		[dir](Vector2f a, Vector2f b)->bool{return dir.dot(a) < dir.dot(b); }) - vertices.begin();
	Vector2f ret = *(vertices.begin() + index);
	return VectorWithIndex(index, Vector2f{ pos(0), pos(1) } + Rotation2Df(pos(2)) * ret);
}

//Intersection gjkSimplex(const UniversalConvexShape& a,
//	const UniversalConvexShape& b)
//{
//
//	Vector2f simplex[3];
//	Vector2f simplex_vertices[6];
//
//	Vector2f dir(b.pos - a.pos);
//	dir.normalize();
//	simplex_vertices[0] = a.gjkSupportVer(dir);
//	simplex_vertices[3] = b.gjkSupportVer(-dir);
//	simplex[0] = simplex_vertices[0] - simplex_vertices[3];
//
//	dir = -simplex[0];
//	dir.normalize();
//
//	simplex_vertices[1] = a.gjkSupportVer(dir);
//	simplex_vertices[4] = b.gjkSupportVer(-dir);
//	simplex[1] = simplex_vertices[1] - simplex_vertices[4];
//	if (dir.dot(simplex[1]) < 0)
//		return { false };
//
//	dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
//	simplex_vertices[2] = a.gjkSupportVer(dir);
//	simplex_vertices[5] = b.gjkSupportVer(-dir);
//	simplex[2] = simplex_vertices[2] - simplex_vertices[5];
//	if (dir.dot(simplex[2]) < 0)
//		return { false };
//
//	while (true)
//	{
//		Vector3f coords = barycentricCoordinates3(simplex, Vector2f{ 0, 0 });
//
//		if (coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0)
//		{
//			//compute shortest distance to each edge
//			coords[0] /= (simplex[1] - simplex[2]).norm();
//			coords[1] /= (simplex[2] - simplex[0]).norm();
//			coords[2] /= (simplex[0] - simplex[1]).norm();
//
//			//choose closest edge
//			int index = 0;
//			if (coords[0] < coords[1])
//				index = 0;
//			else
//				index = 1;
//			
//			if (coords[2] < coords[index])
//				index = 2;
//
//			Vector2f line[2] = { simplex[(index + 2) % 3], simplex[(index + 1) % 3] };
//			Vector2f coords2 = barycentricCoordinates2(line, { 0, 0 });
//			Vector2f a1{ simplex_vertices[(index + 1) % 3] * coords2[0] + simplex_vertices[(index + 2) % 3] * coords2[1] };
//			Vector2f a2{ simplex_vertices[(index + 1) % 3 + 3] * coords2[0] + simplex_vertices[(index + 2) % 3 + 3] * coords2[1] };
//			Vector2f normal;
//			if ((simplex_vertices[(index + 1) % 3] - simplex_vertices[(index + 2) % 3]).squaredNorm() > 
//				(simplex_vertices[(index + 1) % 3 + 3] - simplex_vertices[(index + 2) % 3 + 3]).squaredNorm())
//				normal = simplex_vertices[(index + 2) % 3] - simplex_vertices[(index + 1) % 3];
//			else
//				normal = simplex_vertices[(index + 2) % 3 + 3] - simplex_vertices[(index + 1) % 3 + 3];
//			normal = cross(1, normal);
//			normal.normalize();
//			ClosestFeature cs = pointToTriangle(simplex, { 0, 0 });
//			return { true, a1, normal, cs };
//		}
//		else if (coords[1] < 0)
//		{
//			simplex[1] = simplex[2];
//			simplex_vertices[1] = simplex_vertices[2];
//			simplex_vertices[4] = simplex_vertices[5];
//		}
//		else if (coords[0] < 0)
//		{
//			simplex[0] = simplex[1];
//			simplex_vertices[0] = simplex_vertices[1];
//			simplex_vertices[3] = simplex_vertices[4];
//			simplex[1] = simplex[2];
//			simplex_vertices[1] = simplex_vertices[2];
//			simplex_vertices[4] = simplex_vertices[5];
//		}
//
//		dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
//		simplex_vertices[2] = a.gjkSupportVer(dir);
//		simplex_vertices[5] = b.gjkSupportVer(-dir);
//		simplex[2] = simplex_vertices[2] - simplex_vertices[5];
//		if (dir.dot(simplex[2]) < 0)
//			return false;
//		if ((simplex_vertices[2] == simplex_vertices[0] && simplex_vertices[5] == simplex_vertices[3]) ||
//			(simplex_vertices[2] == simplex_vertices[1] && simplex_vertices[5] == simplex_vertices[4]))
//		{
//			return { false };
//		}
//	}
//
//	return { true };
//}


Intersection gjkSimplex(const UniversalConvexShape& a,
	const UniversalConvexShape& b)
{

	std::vector<Vector2f> simplex{ 3 };
	VectorWithIndex simplex_vertices[6];

	Vector2f dir(b.pos.head<2>() - a.pos.head<2>());
	dir.normalize();
	simplex_vertices[0] = a.gjkSupportVer(dir);
	simplex_vertices[3] = b.gjkSupportVer(-dir);
	simplex[0] = simplex_vertices[0] - simplex_vertices[3];

	dir = -simplex[0];
	dir.normalize();

	simplex_vertices[1] = a.gjkSupportVer(dir);
	simplex_vertices[4] = b.gjkSupportVer(-dir);
	simplex[1] = simplex_vertices[1] - simplex_vertices[4];
	//if (dir.dot(simplex[1]) < 0)
	//{
	//	return Intersection{ false, simplex[1].norm(), {simplex_vertices[1]}, {simplex_vertices[4]} };
	//}
	if (simplex_vertices[0] == simplex_vertices[1] && simplex_vertices[3] == simplex_vertices[4])
	{
		return Intersection{ false, simplex[1].norm(), {simplex_vertices[1]}, {simplex_vertices[4]} };
	}
	assert(simplex_vertices[0] != simplex_vertices[1] || simplex_vertices[3] != simplex_vertices[4]);
	while (true)
	{
		dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
		simplex_vertices[2] = a.gjkSupportVer(dir);
		simplex_vertices[5] = b.gjkSupportVer(-dir);
		simplex[2] = simplex_vertices[2] - simplex_vertices[5];

		if (simplex_vertices[2] == simplex_vertices[0] && simplex_vertices[5] == simplex_vertices[3])
		{
			ClosestFeature cs = pointToLine({ simplex[0], simplex[1] }, { 0, 0 });
			std::vector<VectorWithIndex> aClosestFeature{ cs.feature.size() }, bClosestFeature{ cs.feature.size() };
			for (int i = 0; i < cs.feature.size(); i++)
			{
				aClosestFeature[i] = simplex_vertices[cs.feature[i].index];
				bClosestFeature[i] = simplex_vertices[3 + cs.feature[i].index];
			}
			return Intersection{ false, cs.distance,  aClosestFeature, bClosestFeature };
		}
		if (simplex_vertices[2] == simplex_vertices[1] && simplex_vertices[5] == simplex_vertices[4])
		{
			ClosestFeature cs = pointToLine({ simplex[0], simplex[1] }, { 0, 0 });
			std::vector<VectorWithIndex> aClosestFeature{ cs.feature.size() }, bClosestFeature{ cs.feature.size() };
			for (int i = 0; i < cs.feature.size(); i++)
			{
				aClosestFeature[i] = simplex_vertices[cs.feature[i].index];
				bClosestFeature[i] = simplex_vertices[3 + cs.feature[i].index];
			}
			return Intersection{ false, cs.distance,  aClosestFeature, bClosestFeature };
		}

		Vector3f coords = barycentricCoordinates3(simplex, Vector2f{ 0, 0 });

		if (coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0)
		{
			//compute shortest distance to each edge
			coords[0] /= (simplex[1] - simplex[2]).norm();
			coords[1] /= (simplex[2] - simplex[0]).norm();
			coords[2] /= (simplex[0] - simplex[1]).norm();

			//choose closest edge
			int index = 0;
			if (coords[0] < coords[1])
				index = 0;
			else
				index = 1;

			if (coords[2] < coords[index])
				index = 2;

			Vector2f line[2] = { simplex[(index + 2) % 3], simplex[(index + 1) % 3] };
			Vector2f coords2 = barycentricCoordinates2({ line[0], line[1] }, { 0, 0 });
			Vector2f a1{ simplex_vertices[(index + 1) % 3] * coords2[0] + simplex_vertices[(index + 2) % 3] * coords2[1] };
			Vector2f a2{ simplex_vertices[(index + 1) % 3 + 3] * coords2[0] + simplex_vertices[(index + 2) % 3 + 3] * coords2[1] };
			Vector2f normal;
			if ((simplex_vertices[(index + 1) % 3] - simplex_vertices[(index + 2) % 3]).squaredNorm() >
				(simplex_vertices[(index + 1) % 3 + 3] - simplex_vertices[(index + 2) % 3 + 3]).squaredNorm())
				normal = simplex_vertices[(index + 2) % 3] - simplex_vertices[(index + 1) % 3];
			else
				normal = simplex_vertices[(index + 2) % 3 + 3] - simplex_vertices[(index + 1) % 3 + 3];
			normal = cross(1, normal);
			normal.normalize();
			ClosestFeature cs = pointToTriangle(simplex, { 0, 0 });
			return Intersection{ true, VectorWithIndex{0, a1}, normal };
		}
		else if (coords[1] < 0)
		{
			simplex[1] = simplex[2];
			simplex_vertices[1] = simplex_vertices[2];
			simplex_vertices[4] = simplex_vertices[5];
		}
		else if (coords[0] < 0)
		{
			simplex[0] = simplex[1];
			simplex_vertices[0] = simplex_vertices[1];
			simplex_vertices[3] = simplex_vertices[4];
			simplex[1] = simplex[2];
			simplex_vertices[1] = simplex_vertices[2];
			simplex_vertices[4] = simplex_vertices[5];
		}
	}

	return { true };
}

Vector2f barycentricCoordinates2(std::vector<Vector2f> line, Vector2f p)
{
	Vector2f n = line[1] - line[0];
	float length = n.norm();
	n.normalize();
	Vector2f ret(n.dot(p - line[0]) / length, n.dot(line[1] - p) / length);
	return ret;
}

Vector3f barycentricCoordinates3(std::vector<Vector2f> tri, Vector2f p)
{
	Vector2f v1 = tri[1] - tri[0], v2 = tri[2] - tri[0], v3 = p - tri[0];
	float area = cross(v1, v2);
	if (area == 0)
		return Vector3f(0, 0, 0);
	float area2 = cross(v1, v3), area1 = cross(v3, v2);
	float area0 = area - area1 - area2;
	return Vector3f(area0 / area, area1 / area, area2 / area);
}

Vector2f cartesianCoordinates(const std::vector<Vector2f> tri, const Vector3f bc)
{
	Vector2f ret = bc[0] * tri[0];
	ret += bc[1] * tri[1];
	ret += bc[2] * tri[2];
	return ret;
}

ClosestFeature pointToTriangle(std::vector<Vector2f> tri, const Vector2f& p)
{
	assert(tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2]);
	Vector2f u1 = barycentricCoordinates2({ tri[0], tri[1] }, p);
	Vector2f u2 = barycentricCoordinates2({tri[1], tri[2]}, p);
	Vector2f u3 = barycentricCoordinates2({ tri[2], tri[0] }, p);
	Vector3f v = barycentricCoordinates3({ tri[0], tri[1], tri[2] }, p);

	if (u1(0) <= 0 && u3(1) <= 0) {
		return { {{0, tri[0]}}, (tri[0] - p).norm() };
	}
	else if (u1(1) <= 0 && u2(0) <= 0)
	{
		return { {{1, tri[1]}}, (tri[1] - p).norm() };
	}
	else if (u2(1) <= 0 && u3(0) <= 0)
	{
		return { {{2, tri[2]}}, (tri[2] - p).norm() };
	}
	else if(u1(0) >=0 && u1(1) >= 0 && v(2) <= 0)
	{
		return { {{0, tri[0]}, {1, tri[1]}}, v(2) * cross(tri[1] - tri[0], tri[2] - tri[0]) / (tri[1] - tri[0]).norm() };
	}
	else if (u2(0) >= 0 && u2(1) >= 0 && v(0) <= 0)
	{
		return { {{1, tri[1]}, {2, tri[2]}}, v(0) * cross(tri[1] - tri[0], tri[2] - tri[0]) / (tri[1] - tri[2]).norm() };
	}
	else if (u3(0) >= 0 && u3(1) >= 0 && v(1) <= 0)
	{
		return { {{2, tri[2]}, {0, tri[0]}}, v(1) * cross(tri[1] - tri[0], tri[2] - tri[0]) / (tri[0] - tri[2]).norm() };
	}
	else if(v(0) >= 0 && v(1) >= 0 && v(2) >= 0)
	{
		float area = cross(tri[1] - tri[0], tri[2] - tri[0]);
		v *= area;
		v(0) /= (tri[2] - tri[1]).norm();
		v(1) /= (tri[1] - tri[0]).norm();
		v(2) /= (tri[0] - tri[2]).norm();
		if (v(0) < v(1) && v(0) < v(2))
			return { {{1, tri[1]}, {2, tri[2]}}, v(0) };
		else if (v(1) < v(2) && v(1) < v(0))
			return { {{2, tri[2]}, {0, tri[0]}}, v(1) };
		else if (v(2) < v(1) && v(2) < v(0))
			return { {{0, tri[0]}, {1, tri[1]}}, v(2) };
	}
}

ClosestFeature pointToLine(std::vector<Vector2f> line, const Vector2f& p)
{
	Vector2f u = barycentricCoordinates2(line, p);
	if (u(0) <= 0 && u(1) > 0)
	{
		return ClosestFeature{ {VectorWithIndex{0, line[0]}}, (line[0] - p).norm() };
	}
	else if (u(1) <= 0 && u(0) > 0)
	{
		return ClosestFeature{ {VectorWithIndex{1, line[1]}}, (line[1] - p).norm() };
	}
	else
	{
		return ClosestFeature{ {VectorWithIndex{0, line[0]}, VectorWithIndex{1, line[1]}},
			abs(cross(line[1] - line[0], p - line[0])) / (line[1] - line[0]).norm() };
	}
}

bool collisionDetectionWide(const UniversalConvexShape& a, const UniversalConvexShape& b)
{
	if (a.sphere_bound + b.sphere_bound > (a.pos.head<2>() - b.pos.head<2>()).norm())
		return true;
	else
		return false;
}

Intersection collisionDetection(const UniversalConvexShape& a, const UniversalConvexShape& b)
{
	if (collisionDetectionWide(a, b))
	{
		Intersection r = gjkSimplex(a, b);
	}
	else
	{
		Intersection r{ false, 0, {}, {} };
	}
	return Intersection();
}

Intersection::Intersection(bool a)
{
	if (a)
	{
		collision = true;
		point = VectorWithIndex{ 0, { 0, 0 } };
		normal = { 0, 0 };
	}
	else
	{
		collision = false;
		distance = 1e9;
		aClosestFeature = {};
		bClosestFeature = {};
	}
}

Intersection::Intersection(const Intersection& a)
{
	if (a.collision)
	{
		collision = true;
		point = a.point;
		normal = a.normal;
	}
	else
	{
		collision = false;
		distance = a.distance;
		aClosestFeature = a.aClosestFeature;
		bClosestFeature = a.bClosestFeature;
	}
}


Intersection& Intersection::operator=(const Intersection& a)
{
	if (a.collision)
	{
		if (!collision)
		{
			aClosestFeature.~vector();
			bClosestFeature.~vector();
		}
		collision = true;
		point = a.point;
		normal = a.normal;
	}
	else
	{
		
		if (collision)
		{
			new(&aClosestFeature)std::vector<VectorWithIndex>{a.aClosestFeature};
			new(&bClosestFeature)std::vector<VectorWithIndex>{a.bClosestFeature};
		}
		else
		{
			aClosestFeature = a.aClosestFeature;
			bClosestFeature = a.bClosestFeature;
		}
		collision = false;
		distance = a.distance;
	}
	return *this;
}

Intersection::~Intersection()
{
	if (collision)
	{
		point.~VectorWithIndex();
		normal.~Vector2f();
	}
	else
	{
		aClosestFeature.~vector();
		bClosestFeature.~vector();
	}
}

