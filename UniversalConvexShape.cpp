#include "UniversalConvexShape.h"

float ProjektConvexBody::gravity = 5e-4;

void ProjektConvexPolygon::setTranslation(const Vector2f& _t)
{
	pos.head<2>() = _t;
	tm.setIdentity();
	tm.translate(pos.head<2>());
	tm.rotate(pos(2));
}

void ProjektConvexPolygon::setRotation(float _r)
{
	pos(2) = _r;
	tm.setIdentity();
	tm.translate(pos.head<2>());
	tm.rotate(pos(2));
}

void ProjektConvexPolygon::setPosition(const Vector3f& _pos)
{
	pos = _pos;
	tm.setIdentity();
	tm.translate(pos.head<2>());
	tm.rotate(pos(2));
}

bool ProjektConvexPolygon::addVertex(int i, Vector2f v)
{
	//todo: mechanism checking if new polygon is convex
	vs[i] = v;
	return true;
}

bool ProjektConvexPolygon::addVertex(VectorWithIndex v)
{
	return addVertex(v.index, Vector2f(v));
}

Vector2f ProjektConvexPolygon::getPosition() const
{
	return pos.head<2>();
}

Vector2f ProjektConvexPolygon::getVertex(int i) const
{
	return vs[i];
}

Vector2f ProjektConvexPolygon::getTransformedVertex(int i) const
{

	return tm.linear() * vs[i];
}

Vector2f ProjektConvexPolygon::getVertexPos(int i) const
{
	return tm * vs[i];
}

void ProjektConvexPolygon::offsetVertices(Vector2f _v)
{
	for (Vector2f& vi : vs) vi += _v;
}

VectorWithIndex ProjektConvexPolygon::gjkSupportVer(const Vector2f& dir_) const
{
	Vector2f dir{ Rotation2Df(-pos(2)) * dir_ };
	int index = std::max_element(vs.begin(), vs.end(),
		[dir](Vector2f a, Vector2f b)->bool {return dir.dot(a) < dir.dot(b); }) - vs.begin();
	Vector2f ret = *(vs.begin() + index);
	return VectorWithIndex(index, Vector2f{ pos(0), pos(1) } + Rotation2Df(pos(2)) * ret);
}

//Intersection gjkSimplex(const ProjektConvexPolygon& a,
//	const ProjektConvexPolygon& b)
//{
//	std::array<Vector2f, 3> simplex;
//	std::array<VectorWithIndex, 6> simplex_vertices;
//
//	Vector2f dir(b.pos.head<2>() - a.pos.head<2>());
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
//	if (simplex_vertices[0] == simplex_vertices[1] && simplex_vertices[3] == simplex_vertices[4])
//	{
//		return Intersection{ false, simplex[1].norm(), {simplex_vertices[1]}, {simplex_vertices[4]} };
//	}
//	assert(simplex_vertices[0] != simplex_vertices[1] || simplex_vertices[3] != simplex_vertices[4]);
//	while (true)
//	{
//		dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
//		simplex_vertices[2] = a.gjkSupportVer(dir);
//		simplex_vertices[5] = b.gjkSupportVer(-dir);
//		simplex[2] = simplex_vertices[2] - simplex_vertices[5];
//
//		if (simplex_vertices[2] == simplex_vertices[0] && simplex_vertices[5] == simplex_vertices[3])
//		{
//			ClosestFeature cs = pointToLine({ simplex[0], simplex[1] }, { 0, 0 });
//			std::vector<VectorWithIndex> aClosestFeature{ cs.feature.size() }, bClosestFeature{ cs.feature.size() };
//			for (int i = 0; i < cs.feature.size(); i++)
//			{
//				aClosestFeature[i] = simplex_vertices[cs.feature[i].index];
//				bClosestFeature[i] = simplex_vertices[3 + cs.feature[i].index];
//			}
//			return Intersection{ false, cs.distance,  aClosestFeature, bClosestFeature };
//		}
//		if (simplex_vertices[2] == simplex_vertices[1] && simplex_vertices[5] == simplex_vertices[4])
//		{
//			ClosestFeature cs = pointToLine({ simplex[0], simplex[1] }, { 0, 0 });
//			std::vector<VectorWithIndex> aClosestFeature{ cs.feature.size() }, bClosestFeature{ cs.feature.size() };
//			for (int i = 0; i < cs.feature.size(); i++)
//			{
//				aClosestFeature[i] = simplex_vertices[cs.feature[i].index];
//				bClosestFeature[i] = simplex_vertices[3 + cs.feature[i].index];
//			}
//			return Intersection{ false, cs.distance,  aClosestFeature, bClosestFeature };
//		}
//
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
//			Vector2f coords2 = barycentricCoordinates2({ line[0], line[1] }, { 0, 0 });
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
//			return Intersection{ true, VectorWithIndex{0, a1}, normal };
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
//	}
//
//	return { true };
//}

GjkSimplex & gjkSimplex(const ProjektConvexPolygon& a,
	const ProjektConvexPolygon& b, GjkSimplex & is)
{
	Vector2f dir;

	if(is.simpstate != GjkSimplex::uninitialized)
		is.refresh(a, b);

	switch (is.simpstate)
	{
	case GjkSimplex::uninitialized:
		dir = b.getPosition().head<2>() - a.getPosition().head<2>();
		dir.normalize();
		is.simplex_vertices[0] = a.gjkSupportVer(dir);
		is.simplex_vertices[3] = b.gjkSupportVer(-dir);
		is.simplex[0] = is.simplex_vertices[0] - is.simplex_vertices[3];

	case GjkSimplex::point:
		dir = -is.simplex[0];
		dir.normalize();
		is.simplex_vertices[1] = a.gjkSupportVer(dir);
		is.simplex_vertices[4] = b.gjkSupportVer(-dir);
		is.simplex[1] = is.simplex_vertices[1] - is.simplex_vertices[4];
		if (is.simplex_vertices[0] == is.simplex_vertices[1] && is.simplex_vertices[3] == is.simplex_vertices[4])
		{
			is.simpstate = GjkSimplex::point;
			is.collision = false;
			return is;
		}
		assert(is.simplex_vertices[0] != is.simplex_vertices[1] || is.simplex_vertices[3] != is.simplex_vertices[4]);
		while (true)
		{
	case GjkSimplex::line:
			dir = cross(cross(is.simplex[1] - is.simplex[0], -is.simplex[0]), is.simplex[1] - is.simplex[0]);
			is.simplex_vertices[2] = a.gjkSupportVer(dir);
			is.simplex_vertices[5] = b.gjkSupportVer(-dir);
			is.simplex[2] = is.simplex_vertices[2] - is.simplex_vertices[5];

			if (is.simplex_vertices[2].index == is.simplex_vertices[0].index && is.simplex_vertices[5].index == is.simplex_vertices[3].index)
			{
				is.simpstate = GjkSimplex::line;
				is.collision = false;
				return is;
			}
			if (is.simplex_vertices[2].index == is.simplex_vertices[1].index && is.simplex_vertices[5].index == is.simplex_vertices[4].index)
			{
				is.simpstate = GjkSimplex::line;
				is.collision = false;
				return is;
			}

	case GjkSimplex::triangle:
			Vector3f coords = barycentricCoordinates3(is.simplex, Vector2f{ 0, 0 });

			if (coords[0] >= 0 && coords[1] >= 0 && coords[2] >= 0)
			{
				//compute shortest distance to each edge
				coords[0] /= (is.simplex[1] - is.simplex[2]).norm();
				coords[1] /= (is.simplex[2] - is.simplex[0]).norm();
				coords[2] /= (is.simplex[0] - is.simplex[1]).norm();

				//choose closest edge
				int index = 0;
				if (coords[0] < coords[1])
					index = 0;
				else
					index = 1;

				if (coords[2] < coords[index])
					index = 2;

				is.simpstate = GjkSimplex::triangle;
				is.collision = true;
				return is;
			}
			else if (coords[1] < 0)
			{
				is.simplex[1] = is.simplex[2];
				is.simplex_vertices[1] = is.simplex_vertices[2];
				is.simplex_vertices[4] = is.simplex_vertices[5];
			}
			else if (coords[0] < 0)
			{
				is.simplex[0] = is.simplex[1];
				is.simplex_vertices[0] = is.simplex_vertices[1];
				is.simplex_vertices[3] = is.simplex_vertices[4];
				is.simplex[1] = is.simplex[2];
				is.simplex_vertices[1] = is.simplex_vertices[2];
				is.simplex_vertices[4] = is.simplex_vertices[5];
			}
			else
			{
				is.simpstate = GjkSimplex::line;
				is.collision = false;
				return is;
			}
		}
	}
	throw std::runtime_error("Error in gjkSimplex");
}



void ProjektConvexBody::createShape()
{
	Gdiplus::GraphicsPath* lpshape = createGraphicsPath(pcp.vs) ;
	shape.AddPath(lpshape, false);
	delete lpshape;
}

ProjektConvexBody::ProjektConvexBody() :
	pos(0, 0, 0), vel(0.f, 0.f, 0.f), mass(1), inertia(std::numeric_limits<float>::infinity()),
	shape(Gdiplus::FillModeWinding), sphere_bound(0) {}

ProjektConvexBody::ProjektConvexBody(const ProjektConvexBody& ucs) :
	pos(ucs.pos), vel(ucs.vel), pcp(ucs.pcp), mass(ucs.mass),
		inertia(ucs.inertia), shape(ucs.shape.GetFillMode()), sphere_bound(ucs.sphere_bound), kmat(ucs.kmat)

{
	Gdiplus::PointF* points = new Gdiplus::PointF[ucs.shape.GetPointCount()]();
	BYTE* pointTypes = new BYTE[ucs.shape.GetPointCount()]();
	ucs.shape.GetPathPoints(points, ucs.shape.GetPointCount());
	ucs.shape.GetPathTypes(pointTypes, ucs.shape.GetPointCount());
	Gdiplus::GraphicsPath tmp(points, pointTypes, ucs.shape.GetPointCount(), ucs.shape.GetFillMode());
	shape.AddPath(&tmp, true);
	delete[]points;
	delete[]pointTypes;
}

ProjektConvexBody::ProjektConvexBody(const Vector3f& _pos, const Vector3f& _vel,
	const std::vector<Vector2f>& _vs = {}, float _mass = 1.f, float _inertia = 1.f) :
	pos(_pos), vel(_vel), pcp(_vs), mass(_mass), inertia(_inertia)
{
	Vector2f c = findCenterOfMass(pcp.vs);
	pos.head<2>() += c;
	pcp.setPosition(pos);
	pcp.offsetVertices(-c);
	sphere_bound = std::max_element(pcp.vs.begin(), pcp.vs.end(),
		[](const Vector2f& a, const Vector2f& b)->bool
		{return a.squaredNorm() < b.squaredNorm(); })->norm() * 1.05;
	if (mass == std::numeric_limits<float>::infinity())
		kmat.diagonal() << 0.f, 0.f, 0.f;
	else if(inertia == std::numeric_limits<float>::infinity())
		kmat.diagonal() << 1 / mass, 1 / mass, 0.f;
	else
	{
		inertia = findMomentOfInertia(pcp.vs) * mass;
		kmat.diagonal() << 1 / mass, 1 / mass, 1 / inertia;
	}
	createShape();
}

//Vector2f ProjektConvexBody::getTransformedVertex(int i)
//{
//	Affine2f trans1{};
//	trans1.setIdentity();
//	return trans1.rotate(pos(2)).linear() * vertices[i];
//}
//
//Vector2f ProjektConvexBody::getVertexPos(int i)
//{
//return pos.head<2>() + getTransformedVertex(i);
//}

void ProjektConvexBody::draw(Gdiplus::Graphics* lpgraphics, Gdiplus::Pen* lppen, Gdiplus::Brush* lpbrush)
{
	Gdiplus::Matrix matrix;
	matrix.Translate(pos[0], pos[1]);
	matrix.Rotate(double(this->pos(2)*180.0/3.14), Gdiplus::MatrixOrderPrepend);
	lpgraphics->SetTransform(&matrix);
	if(lpbrush != nullptr)
		lpgraphics->FillPath(lpbrush, &shape);
	if(lppen != nullptr)
		lpgraphics->DrawPath(lppen, &shape);
	lpgraphics->ResetTransform();
}

void ProjektConvexBody::updateVel(float dt)
{
	vel = vel + Vector3f( 0.f, gravity, 0.f ) *dt;
}

void ProjektConvexBody::updatePos(float dt)
{
	pos = pos + vel * dt;
	pcp.setPosition(pos);
}

//Intersection gjkSimplex(const ProjektConvexBody& a,
//	const ProjektConvexBody& b)
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

Vector2f barycentricCoordinates2(std::array<Vector2f, 2> line, Vector2f p)
{
	Vector2f n = line[1] - line[0];
	float length = n.norm();
	n.normalize();
	Vector2f ret(n.dot(p - line[0]) / length, n.dot(line[1] - p) / length);
	return ret;
}

Vector3f barycentricCoordinates3(std::array<Vector2f, 3> tri, Vector2f p)
{
	Vector2f v1 = tri[1] - tri[0], v2 = tri[2] - tri[0], v3 = p - tri[0];
	float area = cross(v1, v2);
	if (area == 0)
		return Vector3f(0, 0, 0);
	float area2 = cross(v1, v3), area1 = cross(v3, v2);
	float area0 = area - area1 - area2;
	return Vector3f(area0 / area, area1 / area, area2 / area);
}

Vector2f cartesianCoordinates(const std::array<Vector2f, 3> tri, const Vector3f bc)
{
	Vector2f ret = bc[0] * tri[0];
	ret += bc[1] * tri[1];
	ret += bc[2] * tri[2];
	return ret;
}

ClosestFeature pointToTriangle(std::array<Vector2f, 3> tri, const Vector2f& p)
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

ClosestFeature pointToLine(std::array<Vector2f, 2> line, const Vector2f& p)
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

bool collisionDetectionWide(const ProjektConvexBody& a, const ProjektConvexBody& b)
{
	if (a.sphere_bound + b.sphere_bound > (a.pos.head<2>() - b.pos.head<2>()).norm())
		return true;
	else
		return false;
}

Vector2f findCenterOfMass(const std::vector<Vector2f> &_vs)
{
	Vector2f c{ 0, 0 };
	float area = 0;
	for (int i = 0; i < _vs.size(); i++)
	{
		float tmp = cross(_vs[i], _vs[(i + 1) % _vs.size()]);
		c += (_vs[i] + _vs[(i + 1) % _vs.size()]) * tmp / 6;
		area += tmp;
	}
	return c/area;
}

float findMomentOfInertia(const std::vector<Vector2f>& _vs)
{
	float nom = 0, den = 0;
	auto _n = [n = _vs.size()](int i)->int{return (i + 1) % n; };
	for (int i = 0; i < _vs.size(); i++)
	{
		nom += cross(_vs[_n(i)], _vs[i]) *
			(_vs[i].dot(_vs[i]) + 
				_vs[i].dot(_vs[_n(i)]) +
					_vs[_n(i)].dot(_vs[_n(i)]));
		den += cross(_vs[_n(i)], _vs[i]);
	}
	den *= 6;
	return nom / den;
}

Gdiplus::GraphicsPath* createGraphicsPath(const std::vector<Vector2f>& _vs)
{
	Gdiplus::GraphicsPath* shape = new Gdiplus::GraphicsPath();
	shape->AddLine(0.0f, 0.0f, _vs[0](0), _vs[0](1));
	shape->StartFigure();
	for (int i = 1; i < _vs.size(); i++)
	{
		Vector2f v1{ _vs[i - 1] };
		Vector2f v2{ _vs[i] };
		v1 += 0 * v1.normalized();
		v2 += 0 * v2.normalized();

		shape->AddLine(v1[0], v1[1], v2[0], v2[1]);
	}
	Vector2f v1{ *(_vs.end() - 1) }, v2{ _vs[0] };
	v1 += 0.0 * v1.normalized();
	v2 += 0.0 * v2.normalized();
	shape->AddLine(v1[0], v1[1], v2[0], v2[1]);
	shape->CloseFigure();
	return shape;
}

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

void GjkSimplex::refresh(const ProjektConvexPolygon & _a, const ProjektConvexPolygon & _b)
{
	int w;
	switch (simpstate)
	{
	default:
	case uninitialized:
		w = 0;
		break;
	case point:
		w = 1;
		break;
	case line:
		w = 2;
		break;
	case triangle:
		w = 3;
		break;
	}

	for (std::array<VectorWithIndex, 6>::iterator it = simplex_vertices.begin(); it != simplex_vertices.begin() + w; it++)
	{
		*it = VectorWithIndex(it->index, _a.getVertexPos(it->index));
		*(it + 3) = VectorWithIndex((it + 3)->index, _b.getVertexPos((it + 3)->index));
	}

	for (int i = 0; i < 3; i++)
		simplex[i] = simplex_vertices[i] - simplex_vertices[i + 3];
}
