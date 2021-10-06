#include "Triangle.h"

PointF Triangle::gravity = { 0, 2e-5 };

REAL inner_product(const PointF& p1, const PointF& p2)
{
	return p1.X * p2.X + p1.Y * p2.Y;
}

Triangle::Triangle(PointF pos_, REAL size_, REAL initial_rotation_, PointF vel_, REAL omega_) :
	pos(pos_), size(size_), vel(vel_), omega(omega_)
{
	Gdiplus::Matrix m0, m;
	m0.Rotate(initial_rotation_);
	PointF v(0, -size);
	m0.TransformVectors(&v);
	m.Rotate(120);
	ver[0] = v;
	m.TransformVectors(&v);
	ver[1] = v;
	m.TransformVectors(&v);
	ver[2] = v;

	mass = 1;
	inertia = mass * size * size / 12 ;
	bounding_sphere = 1.1 * size;
}

void Triangle::draw(Graphics* lpgraphics_, Pen* lppen_)
{

	lpgraphics_->DrawLine(lppen_, pos + ver[0], pos + ver[1]);
	lpgraphics_->DrawLine(lppen_, pos + ver[1], pos + ver[2]);
	lpgraphics_->DrawLine(lppen_, pos + ver[2], pos + ver[0]);
}

void Triangle::update(REAL dt)
{
	Gdiplus::Matrix m1;
	pos = pos + vel * dt - gravity * 0.5 * dt * dt;
	vel = vel + gravity * dt;
	m1.RotateAt(180 / 3.14 * omega * dt, pos);
	m1.TransformVectors(ver, 3);

}

void Triangle::update(REAL dt, PointF* relative_vector)
{
	Gdiplus::Matrix m1;
	m1.RotateAt(180 / 3.14 * omega * dt, pos);
	m1.TransformVectors(relative_vector);
}



void Triangle::collision_with_wall(const RectF* walls)
{
	for (PointF* r = ver; r < ver + 3; r++)
	{
		if (pos.Y + r->Y >= walls->GetBottom() || pos.X + r->X <= walls->GetLeft() || pos.X + r->X >= walls->GetRight())
		{
			if (pos.Y + r->Y >= walls->GetBottom())
				pos.Y += (walls->GetBottom() - r->Y - pos.Y) * 1;
			else if (pos.X + r->X <= walls->GetLeft())
				pos.X += (walls->GetLeft() - r->X - pos.X) * 1;
			else if (pos.X + r->X >= walls->GetRight())
				pos.X += (walls->GetRight() - r->X - pos.X) * 1;

			collision_with_static_figure(vel, omega, *r);
		}

	}

}

void Triangle::collision_with_figure(Triangle& tri, REAL dt)
{
	PointF res1[2], res2[2];
	BOOL flag1 = FALSE, flag2 = FALSE;

	if (sqrt(inner_product(tri.pos - pos, tri.pos - pos)) < bounding_sphere + tri.bounding_sphere)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (!flag1)
				{
					if (intersect_segments(pos + ver[i], pos + ver[(i + 1) % 3], tri.pos + tri.ver[j], tri.pos + tri.ver[(j + 1) % 3], res1))
						flag1 = TRUE;
				}
				else
				{
					if (intersect_segments(pos + ver[i], pos + ver[(i + 1) % 3], tri.pos + tri.ver[j], tri.pos + tri.ver[(j + 1) % 3], res2))
						flag2 = TRUE;
				}

				if (flag1 && flag2) break;
			}
			if (flag1 && flag2) break;
		}
		if (flag1 && flag2)
		{
			PointF res = (res1[0] + res2[0]) * 0.5;
			PointF sur = res1[0] - res2[0];
			REAL tmp = sur.X;
			sur.X = -sur.Y;
			sur.Y = tmp;
			if (inner_product(sur, pos - tri.pos) < 0)
				sur = sur * (-1);


			PointF r1 = res - pos, r2 = res - tri.pos;

			REAL v01x = vel.X, v01y = vel.Y, v02x = tri.vel.X, v02y = tri.vel.Y, omega01 = omega, omega02 = tri.omega;
			collision_with_figure2(vel, omega, r1, tri.vel, tri.omega, r2, tri, sur);

		}
	}
}

void Triangle::collision_with_figure2(PointF& vel1, REAL& omega1, const PointF& r1, PointF& vel2, REAL& omega2, const PointF& r2, \
	const Triangle& tri, const PointF& sur)
{
	Vector2f normal{ sur.X, sur.Y };
	normal.normalize();
	MatrixXf jmat(1, 6);
	//jmat << normal[0], normal[1], normal[1] * r1.X - normal[0] * r1.Y, -normal[0], -normal[1], normal[0] * r2.Y - normal[1] * r2.X;
	jmat << normal[0], normal[1], normal[1] * r1.X - normal[0] * r1.Y, -normal[0], -normal[1], normal[0] * r2.Y - normal[1] * r2.X;
	float jminvjt = (jmat(0, 0) * jmat(0, 0) + jmat(0, 1) * jmat(0, 1)) / mass +
		jmat(0, 2) * jmat(0, 2) / inertia +
		(jmat(0, 3) * jmat(0, 3) + jmat(0, 4) * jmat(0, 4)) / tri.mass +
		jmat(0, 5) * jmat(0, 5) / tri.inertia;
	float lambda = -1 / jminvjt * (jmat(0, 0) * vel1.X + jmat(0, 1) * vel1.Y + jmat(0, 2) * omega1 + 
		jmat(0, 3) * vel2.X + jmat(0, 4) * vel2.Y + jmat(0, 5) * omega2);
	if (lambda > 0)
	{
		vel1.X += jmat(0, 0) * lambda / mass;
		vel1.Y += jmat(0, 1) * lambda / mass;
		omega1 += jmat(0, 2) * lambda / inertia;
		vel2.X += jmat(0, 3) * lambda / tri.mass;
		vel2.Y += jmat(0, 4) * lambda / tri.mass;
		omega2 += jmat(0, 5) * lambda / tri.inertia;
	}
}


void Triangle::collision_with_static_figure(PointF& vel, REAL& omega, const PointF& r)
{
	PointF v0 = vel;
	REAL omega0 = omega;
	REAL temp1 = mass * pow(r.X, 2) + mass * pow(r.Y, 2) + inertia;
	vel.X = r.Y * (mass * r.Y * v0.X - mass * r.X * v0.Y + inertia * omega0) / temp1;
	vel.Y = r.X * (-mass * r.Y * v0.X + mass * r.X * v0.Y - inertia * omega0) / temp1;
	omega = -(-mass * r.Y * v0.X + mass * r.X * v0.Y - inertia * omega0) / temp1;
}


bool Triangle::is_point_int_triangle(const Triangle* tri, const PointF& point)
{
	if (tri->pos.X + std::min_element(tri->ver, tri->ver + 3, [](PointF a, PointF b)->bool {return a.X < b.X; })->X <= point.X &&
		tri->pos.X + std::max_element(tri->ver, tri->ver + 3, [](PointF a, PointF b)->bool {return a.X < b.X; })->X >= point.X &&
		tri->pos.Y + std::min_element(tri->ver, tri->ver + 3, [](PointF a, PointF b)->bool {return a.Y < b.Y; })->Y <= point.Y &&
		tri->pos.Y + std::max_element(tri->ver, tri->ver + 3, [](PointF a, PointF b)->bool {return a.Y < b.Y; })->Y >= point.Y)
	{
		for (int i = 0; i < 3; i++)
		{
			if (intersect_segments(tri->ver[i], tri->ver[(i + 1) % 3], tri->pos, point, nullptr))
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

BOOL Triangle::intersect_segments(IN PointF a1, IN PointF b1, IN PointF a2, IN PointF b2, OUT PointF  res[2])
{

	if (!((a1.X < a2.X && a1.X < b2.X && b1.X < a2.X && b1.X < b2.X) || (a1.X > a2.X && a1.X > b2.X && b1.X > a2.X && b1.X > b2.X)) &&
		!((a1.Y < a2.Y && a1.Y < b2.Y && b1.Y < a2.Y && b1.Y < b2.Y) || (a1.Y > a2.Y && a1.Y > b2.Y && b1.Y > a2.Y && b1.Y > b2.Y)))
	{


		PointF v1 = b1 - a1, v2 = b2 - a2, v3 = a2 - a1;
		REAL d1 = -v3.X * v2.Y + v3.Y * v2.X;
		REAL d2 = -v3.X * v1.Y + v3.Y * v1.X;
		REAL d3 = -v1.X * v2.Y + v1.Y * v2.X;
		REAL s, t;

		if (d3 == 0)
		{
			if (d1 == 0)
			{
				if (a1.X > b1.X) std::swap(a1, b1);
				if (a2.X > b2.X) std::swap(a2, b2);
				if (res)
				{
					res[0] = (a1.X < a2.X) ? a2 : a1;
					res[1] = (b1.X < b2.X) ? b2 : b1;
				}

				return TRUE;

			}
			else
			{
				return FALSE;
			}
		}
		else
		{
			s = d1 / d3;
			t = d2 / d3;


			if (0 < s && s < 1 && 0 < t && t < 1)
			{
				if (res)
					res[0] = a1 + (b1 - a1) * s;
				return TRUE;
			}
			else
			{
				return FALSE;
			}
		}
	}
	else
	{
		return FALSE;
	}

}