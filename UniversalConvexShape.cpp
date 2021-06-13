#include "UniversalConvexShape.h"

Vector2f UniversalConvexShape::gravity{ 0, 1e-5 };

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
	return Vector2f(-v1*v2[1], v1*v2[0]);
}

UniversalConvexShape::UniversalConvexShape() :
	pos(200, 200), vel(0, 0), vertices(), mass(1), inertia(1), omega(0),
	rot(0), shape(FillModeWinding), sphere_bound(10) {}

UniversalConvexShape::UniversalConvexShape(const UniversalConvexShape& ucs) :
	pos(ucs.pos), vel(ucs.vel), vertices(ucs.vertices), mass(ucs.mass),
		inertia(ucs.inertia), omega(ucs.omega), rot(ucs.rot), shape(ucs.shape.GetFillMode()), sphere_bound(ucs.sphere_bound)

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

void UniversalConvexShape::draw(Graphics* lpgraphics, Pen* lppen, Brush* lpbrush)
{
	Gdiplus::Matrix matrix;
	matrix.Translate(pos[0], pos[1]);
	matrix.Rotate(this->rot*180/3.14, MatrixOrderPrepend);
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
		shape.AddLine(vertices[i - 1][0], vertices[i - 1][1], \
			vertices[i][0], vertices[i][1]);
	}
	shape.AddLine(vertices[vertices.size() - 1][0], vertices[vertices.size() - 1][1], \
		vertices[0][0], vertices[0][1]);

	this->sphere_bound = std::max_element(vertices.begin(), vertices.end(), \
		[](Vector2f a, Vector2f b)->bool {return a.squaredNorm() < b.squaredNorm(); })->norm();
	spdlog::get("basic_logger")->info("sphere_bound {}", sphere_bound);
}

void UniversalConvexShape::update(float dt)
{
	pos = pos + vel * dt - gravity * 0.5 * dt * dt;
	vel = vel + gravity * dt;
	rot += omega * dt;
}

Vector2f UniversalConvexShape::gjkSupportVer(const Vector2f& dir_) const 
{
	Affine2f trans1{};
	trans1.setIdentity();
	Vector2f dir{ dir_ };
	dir = trans1.rotate(-rot).linear() * dir;
	Vector2f ret = *std::max_element(vertices.begin(), vertices.end(), 
		[dir](Vector2f a, Vector2f b)->bool{return dir.dot(a) < dir.dot(b); });
	trans1.setIdentity();
	return pos + trans1.rotate(rot).linear() * ret;
}

void UniversalConvexShape::collisionWithRect(const Gdiplus::RectF& rect, float dt)
{
	Intersection intersection = collisionWithPlane({ 0, rect.GetBottom() }, { 0, 1 });
	if (intersection.collision)
	{
		Vector2f penetration = intersection.point - Vector2f{0, rect.GetBottom()};
		penetration[0] = 0;
		penetration = penetration/dt;
		spdlog::get("basic_logger")->info("penetration: ({:.2f}, {:.2f})", penetration[0], penetration[1]);
		collisionWithFixedObject(vel, omega, intersection.point - pos, penetration);
		
	}
	intersection = collisionWithPlane({ rect.GetLeft() , 0}, { -1, 0 });
	if (intersection.collision)
	{
		collisionWithFixedObject(vel, omega, intersection.point - pos, { 0, 0 });
	}
	intersection = collisionWithPlane({ rect.GetRight(), 0 }, { 1, 0 });
	if (intersection.collision)
	{
		collisionWithFixedObject(vel, omega, intersection.point - pos, { 0, 0 });
	}
	
}

Intersection UniversalConvexShape::collisionWithPlane(Vector2f position, Vector2f dir)
{
	dir.normalize();
	
	if (dir.dot(position) < dir.dot(pos) + sphere_bound)
	{
		Vector2f supportVer = gjkSupportVer(dir);
		spdlog::get("basic_logger")->info("{} < {} + ({:.2f}, {:.2f})", dir.dot(position), dir.dot(pos), supportVer[0], supportVer[1]);
		float tmp = dir.dot(supportVer) - dir.dot(position);
		if (tmp > 0)
		{
			pos -= tmp * dir;
			return { true, supportVer };
		}
		else
		{
			return { false };
		}
	}
	else
	{
		return { false };
	}
}

Intersection gjkSimplex(const UniversalConvexShape& a,
	const UniversalConvexShape& b)
{

	Vector2f simplex[3];
	Vector2f simplex_vertices[6];

	Vector2f dir(a.pos - b.pos);
	dir.normalize();
	simplex_vertices[0] = a.gjkSupportVer(dir);
	simplex_vertices[3] = b.gjkSupportVer(-dir);
	simplex[0] = simplex_vertices[0] - simplex_vertices[3];

	dir = -simplex[0];
	dir.normalize();

	simplex_vertices[1] = a.gjkSupportVer(dir);
	simplex_vertices[4] = b.gjkSupportVer(-dir);
	simplex[1] = simplex_vertices[1] - simplex_vertices[4];
	if (dir.dot(simplex[1]) < 0)
		return { false };

	dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
	simplex_vertices[2] = a.gjkSupportVer(dir);
	simplex_vertices[5] = b.gjkSupportVer(-dir);
	simplex[2] = simplex_vertices[2] - simplex_vertices[5];
	if (dir.dot(simplex[2]) < 0)
		return { false };

	while (true)
	{
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
			Vector2f coords2 = barycentricCoordinates2(line, { 0, 0 });
			Vector2f a1{ simplex_vertices[(index + 1) % 3] * coords2[0] + simplex_vertices[(index + 2) % 3] * coords2[1] };
			Vector2f a2{ simplex_vertices[(index + 1) % 3 + 3] * coords2[0] + simplex_vertices[(index + 2) % 3 + 3] * coords2[1] };

			return { true, a1 };
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

		dir = cross(cross(simplex[1] - simplex[0], -simplex[0]), simplex[1] - simplex[0]);
		simplex_vertices[2] = a.gjkSupportVer(dir);
		simplex_vertices[5] = b.gjkSupportVer(-dir);
		simplex[2] = simplex_vertices[2] - simplex_vertices[5];
		if (dir.dot(simplex[2]) < 0)
			return false;
		if ((simplex_vertices[2] == simplex_vertices[0] && simplex_vertices[5] == simplex_vertices[3]) ||
			(simplex_vertices[2] == simplex_vertices[1] && simplex_vertices[5] == simplex_vertices[4]))
		{
			return { false };
		}
	}

	return { true };
}

void UniversalConvexShape::collisionWithFixedObject(Vector2f& vel, float& omega, const Vector2f& r, const Vector2f& p)
{
	MatrixXf matrix(5, 5);
	matrix << mass, 0, 0, -1, 0,
		0, mass, 0, 0, -1,
		0, 0, inertia, r[1], -r[0],
		1, 0, -r[1], 0, 0,
		0, 1, r[0], 0, 0;
	MatrixXf rhs(5, 1), lhs(5, 1);
	rhs << mass * vel[0], mass* vel[1], inertia* omega, -p[0], -p[1];
	lhs = matrix.householderQr().solve(rhs);

	Vector2f momentum_change(lhs(3), lhs(4));
	if (r.dot(momentum_change) < 0)
	{
		vel[0] = lhs(0);
		vel[1] = lhs(1);
		omega = lhs(2);
	}
}

void collisionWithMovingObjectSol(float mass1, float inertia1, Vector2f& vel1, float& omega1, Vector2f r1,
	float mass2, float inertia2, Vector2f& vel2, float& omega2, Vector2f r2)
{
	gsl_matrix* equations = gsl_matrix_alloc(8, 8);
	gsl_vector* yvector = gsl_vector_alloc(8);
	gsl_vector* xvector = gsl_vector_alloc(8);

	gsl_matrix_set_zero(equations);
	gsl_matrix_set(equations, 0, 0, mass1); gsl_matrix_set(equations, 0, 6, -1);
	gsl_matrix_set(equations, 1, 1, mass1); gsl_matrix_set(equations, 1, 7, -1);
	gsl_matrix_set(equations, 2, 2, mass2); gsl_matrix_set(equations, 2, 6, 1);
	gsl_matrix_set(equations, 3, 3, mass2); gsl_matrix_set(equations, 3, 7, 1);
	gsl_matrix_set(equations, 4, 4, -inertia1); gsl_matrix_set(equations, 4, 6, -r1[1]); gsl_matrix_set(equations, 4, 7, r1[0]);
	gsl_matrix_set(equations, 5, 5, -inertia2); gsl_matrix_set(equations, 5, 6, r2[1]); gsl_matrix_set(equations, 5, 7, -r2[0]);
	gsl_matrix_set(equations, 6, 0, 1); gsl_matrix_set(equations, 6, 2, -1); gsl_matrix_set(equations, 6, 4, -r1[1]); gsl_matrix_set(equations, 6, 5, r2[1]);
	gsl_matrix_set(equations, 7, 1, 1); gsl_matrix_set(equations, 7, 3, -1); gsl_matrix_set(equations, 7, 4, r1[0]); gsl_matrix_set(equations, 7, 5, -r2[0]);

	gsl_vector_set(yvector, 0, mass1 * vel1[0]);
	gsl_vector_set(yvector, 1, mass1 * vel1[1]);
	gsl_vector_set(yvector, 2, mass2 * vel2[0]);
	gsl_vector_set(yvector, 3, mass2 * vel2[1]);
	gsl_vector_set(yvector, 4, -inertia1 * omega1);
	gsl_vector_set(yvector, 5, -inertia2 * omega2);
	gsl_vector_set(yvector, 6, 0);
	gsl_vector_set(yvector, 7, 0);

	gsl_vector* tau = gsl_vector_alloc(8);

	gsl_linalg_QR_decomp(equations, tau);
	gsl_linalg_QR_solve(equations, tau, yvector, xvector);

	Vector2f delta_momentum(0, 0);
	delta_momentum[0] = gsl_vector_get(xvector, 6);
	delta_momentum[1] = gsl_vector_get(xvector, 7);

	if (delta_momentum.dot(r2 - r1) > 0)
	{
		vel1[0] = gsl_vector_get(xvector, 0);
		vel1[1] = gsl_vector_get(xvector, 1);
		vel2[0]= gsl_vector_get(xvector, 2);
		vel2[1] = gsl_vector_get(xvector, 3);
		omega1 = gsl_vector_get(xvector, 4);
		omega2 = gsl_vector_get(xvector, 5);
	}

	gsl_matrix_free(equations);
	gsl_vector_free(xvector);
	gsl_vector_free(yvector);
	gsl_vector_free(tau);
}

//void collisionWithMovingObjectSol(float mass1, float inertia1, Vector2f& vel1, float& omega1, Vector2f r1,
//	float mass2, float inertia2, Vector2f& vel2, float& omega2, Vector2f r2)
//{
//	MatrixXf equations(6, 6);
//	equations <<
//		mass1, 0, 0, 0, -1, 0,
//		0, mass1, 0, 0, 0, -1,
//		0, 0, mass2, 0, 1, 0,
//		0, 0, 0, mass2, 0, 1,
//		1, 0, -1, 0, 0, 0,
//		0, 1, 0, -1, 0, 0;
//
//	MatrixXf yvector(6, 1), xvector(6, 1);
//	yvector << mass1 * vel1[0], mass1* vel1[1], mass2* vel2[0], mass2* vel2[1], 0, 0;
//	xvector = equations.householderQr().solve(yvector);
//
//	Vector2f delta_momentum(xvector(4), xvector(5));
//
//	if (delta_momentum.dot(r2 - r1))
//	{
//		vel1[0] = xvector(0);
//		vel1[1] = xvector(1);
//		vel2[0] = xvector(2);
//		vel2[1] = xvector(3);
//	}
//
//}

void collisionWithMovingObject(UniversalConvexShape& a, UniversalConvexShape& b)
{
	Intersection intersection = collisionDetection(a, b);
	if (intersection.collision)
	{
		collisionWithMovingObjectSol(a.mass, a.inertia, a.vel, a.omega, intersection.point - a.pos,
			b.mass, b.inertia, b.vel, b.omega, intersection.point - b.pos);
	}
}

Vector2f barycentricCoordinates2(const Vector2f line[2], const Vector2f p)
{
	Vector2f n = line[1] - line[0];
	float length = n.norm();
	n.normalize();
	Vector2f ret(n.dot(p - line[0]) / length, n.dot(line[1] - p) / length);
	return ret;
}

Vector3f barycentricCoordinates3(const Vector2f tri[3], const Vector2f p)
{
	Vector2f v1 = tri[1] - tri[0], v2 = tri[2] - tri[0], v3 = p - tri[0];
	float area = cross(v1, v2);
	if (area == 0)
		return Vector3f(0, 0, 0);
	float area2 = cross(v1, v3), area1 = cross(v3, v2);
	float area0 = area - area1 - area2;
	return Vector3f(area0 / area, area1 / area, area2 / area);
}

std::vector<Vector2f> pointToTriangle(const Vector2f tri[3], const Vector2f p)
{
	return std::vector<Vector2f>();
}

Intersection collisionDetection(const UniversalConvexShape& a, const UniversalConvexShape& b)
{
	if (a.sphere_bound + b.sphere_bound < (a.pos - b.pos).squaredNorm())
		return gjkSimplex(a, b);
	else
		return { false };
}