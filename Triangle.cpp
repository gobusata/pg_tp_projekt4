#include "Triangle.h"

PointF Triangle::gravity = { 0, 2e-5 };

REAL inner_product(const PointF& p1, const PointF& p2)
{
	return p1.X * p2.X + p1.Y * p2.Y;
}

Triangle::Triangle(PointF pos_, REAL size_, REAL initial_rotation_, PointF vel_, REAL omega_) :
	pos(pos_), size(size_), vel(vel_), omega(omega_)
{
	Matrix m0, m;
	m0.Rotate(initial_rotation_);
	PointF v(0, -size);
	m0.TransformVectors(&v);
	m.Rotate(120);
	ver[0] = v;
	m.TransformVectors(&v);
	ver[1] = v;
	m.TransformVectors(&v);
	ver[2] = v;

}

void Triangle::draw(Graphics* lpgraphics_, Pen* lppen_)
{

	lpgraphics_->DrawLine(lppen_, pos + ver[0], pos + ver[1]);
	lpgraphics_->DrawLine(lppen_, pos + ver[1], pos + ver[2]);
	lpgraphics_->DrawLine(lppen_, pos + ver[2], pos + ver[0]);
}

void Triangle::update(REAL dt)
{
	Matrix m1;
	pos = pos + vel * dt - gravity * 0.5 * dt * dt;
	vel = vel + gravity * dt;
	m1.RotateAt(180 / 3.14 * omega * dt, pos);
	m1.TransformVectors(ver, 3);

}

void Triangle::update(REAL dt, PointF* relative_vector)
{
	Matrix m1;
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
			sur = sur*(-1);
			

		PointF r1 = res - pos, r2 = res - tri.pos;

		REAL v01x = vel.X, v01y = vel.Y, v02x = tri.vel.X, v02y = tri.vel.Y, omega01 = omega, omega02 = tri.omega;
		collision_with_figure2(vel, omega, r1, tri.vel, tri.omega, r2, tri, sur);
	
	}
}

void Triangle::collision_with_figure2(PointF& vel1, REAL& omega1, const PointF& r1, PointF& vel2, REAL& omega2, const PointF& r2, \
	const Triangle& tri, const PointF& sur)
{
	REAL v01x = vel1.X, v01y = vel1.Y, v02x = vel2.X, v02y = vel2.Y, omega01 = omega1, omega02 = omega2;

	/*vel1.X = (mass * tri.mass * (mass * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * v01x + mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass * v01x + tri.inertia * tri.mass * (std::pow(r1.X, 2) * (-(omega02 * r2.Y) + v02x) + std::pow(r1.Y, 2) * v01x + r1.X * r1.Y * (omega02 * r2.X + v02y - v01y))) + inertia * (tri.inertia * (mass + tri.mass) * (tri.mass * (omega01 * r1.Y - omega02 * r2.Y + v02x) + mass * v01x) + mass * tri.mass * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * v01x + tri.mass * (std::pow(r2.X, 2) * (omega01 * r1.Y + v02x) + std::pow(r2.Y, 2) * v01x - r2.X * r2.Y * (omega01 * r1.X - v02y + v01y))))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
	vel1.Y = (mass * tri.mass * (inertia * mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * v01y + mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass * v01y + inertia * tri.mass * (std::pow(r2.Y, 2) * (-(omega01 * r1.X) + v02y) + r2.X * r2.Y * (omega01 * r1.Y + v02x - v01x) + std::pow(r2.X, 2) * v01y)) + tri.inertia * (inertia * std::pow(tri.mass, 2) * (-(omega01 * r1.X) + omega02 * r2.X + v02y) + std::pow(mass, 2) * (inertia + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.mass) * v01y + mass * tri.mass * (inertia * (-(omega01 * r1.X) + omega02 * r2.X + v02y + v01y) + tri.mass * (std::pow(r1.Y, 2) * (omega02 * r2.X + v02y) - r1.X * r1.Y * (omega02 * r2.Y - v02x + v01x) + std::pow(r1.X, 2) * v01y)))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
	vel2.X = (mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass * v02x + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * tri.mass * v02x + mass * tri.inertia * (std::pow(r1.Y, 2) * v02x + std::pow(r1.X, 2) * (omega02 * r2.Y + v01x) + r1.X * r1.Y * (-(omega02 * r2.X) - v02y + v01y))) + inertia * (tri.inertia * (mass + tri.mass) * (tri.mass * v02x + mass * (-(omega01 * r1.Y) + omega02 * r2.Y + v01x)) + mass * tri.mass * ((std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * v02x + mass * (std::pow(r2.Y, 2) * v02x + std::pow(r2.X, 2) * (-(omega01 * r1.Y) + v01x) + r2.X * r2.Y * (omega01 * r1.X - v02y + v01y))))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
	vel2.Y = (mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass * v02y + inertia * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * v02y + inertia * mass * (std::pow(r2.X, 2) * v02y + r2.X * r2.Y * (-(omega01 * r1.Y) - v02x + v01x) + std::pow(r2.Y, 2) * (omega01 * r1.X + v01y))) + tri.inertia * (inertia * std::pow(tri.mass, 2) * v02y + mass * tri.mass * ((std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.mass * v02y + inertia * (omega01 * r1.X - omega02 * r2.X + v02y + v01y)) + std::pow(mass, 2) * (inertia * (omega01 * r1.X - omega02 * r2.X + v01y) + tri.mass * (std::pow(r1.X, 2) * v02y + r1.X * r1.Y * (omega02 * r2.Y - v02x + v01x) + std::pow(r1.Y, 2) * (-(omega02 * r2.X) + v01y))))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
	omega1 = (tri.inertia * (mass + tri.mass) * (inertia * omega01 * tri.mass + mass * (inertia * omega01 + r1.Y * tri.mass * (omega02 * r2.Y - v02x + v01x) + r1.X * tri.mass * (omega02 * r2.X + v02y - v01y))) + mass * tri.mass * (inertia * mass * omega01 * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) + inertia * omega01 * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + mass * (r1.Y * r2.X - r1.X * r2.Y) * tri.mass * (r2.X * (-v02x + v01x) + r2.Y * (-v02y + v01y)))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
	omega2 = (inertia * omega02 * tri.inertia * std::pow(tri.mass, 2) + std::pow(mass, 2) * (inertia * omega02 * tri.inertia + (r1.Y * r2.X - r1.X * r2.Y) * std::pow(tri.mass, 2) * (r1.X * (-v02x + v01x) + r1.Y * (-v02y + v01y)) + tri.mass * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia + inertia * r2.Y * (omega01 * r1.Y + v02x - v01x) + inertia * r2.X * (omega01 * r1.X - v02y + v01y))) + mass * tri.mass * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * tri.mass + inertia * (2 * omega02 * tri.inertia + r2.Y * tri.mass * (omega01 * r1.Y + v02x - v01x) + r2.X * tri.mass * (omega01 * r1.X - v02y + v01y)))) / (inertia * (mass + tri.mass) * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + tri.inertia * (mass + tri.mass)) + mass * tri.mass * (mass * std::pow(r1.Y * r2.X - r1.X * r2.Y, 2) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)));
*/

	/*vel1.X = (std::pow(mass, 2) * tri.inertia * std::pow(tri.mass, 2) * (std::pow(r1.X, 2) * r2.Y * (-0.1 * omega02 * r2.Y + 0.1 * v02x - 0.1 * v01x) + std::pow(r1.Y, 2) * r2.X * (-0.1 * omega02 * r2.X - 0.1 * v02y + 0.1 * v01y) + r1.X * r1.Y * (0.1 * r2.Y * v02y + r2.X * (0.2 * omega02 * r2.Y - 0.1 * v02x + 0.1 * v01x) - 0.1 * r2.Y * v01y)) + std::pow(inertia, 2) * (omega02 * tri.inertia * std::pow(tri.mass, 2) + std::pow(mass, 2) * (omega02 * tri.inertia + r2.Y * tri.mass * (omega01 * r1.Y + v02x - 1. * v01x) + r2.X * tri.mass * (omega01 * r1.X - 1. * v02y + v01y)) + mass * tri.mass * (2. * omega02 * tri.inertia + r2.Y * tri.mass * (omega01 * r1.Y + v02x - 1. * v01x) + r2.X * tri.mass * (omega01 * r1.X - 1. * v02y + v01y))) + inertia * mass * tri.mass * (tri.inertia * tri.mass * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) - 0.1 * omega02 * std::pow(r2.X, 2) - 0.1 * omega02 * std::pow(r2.Y, 2) + r2.Y * (0.1 * omega01 * r1.Y + 0.1 * v02x - 0.1 * v01x) + r2.X * (0.1 * omega01 * r1.X - 0.1 * v02y + 0.1 * v01y)) + mass * (tri.inertia * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) - 0.1 * omega02 * std::pow(r2.X, 2) - 0.1 * omega02 * std::pow(r2.Y, 2) + r2.Y * (0.1 * omega01 * r1.Y + 0.1 * v02x - 0.1 * v01x) + r2.X * (0.1 * omega01 * r1.X - 0.1 * v02y + 0.1 * v01y)) + tri.mass * (std::pow(r1.X, 2) * r2.Y * (v02x - 1. * v01x) + r1.X * r1.Y * (r2.X * (-1. * v02x + v01x) + r2.Y * (v02y - 1. * v01y)) + std::pow(r1.Y, 2) * r2.X * (-1. * v02y + v01y))))) / (inertia * (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2)))));
	vel1.Y = (mass * tri.mass * (inertia * mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * v01y + mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass * v01y + inertia * tri.mass * (r2.X * r2.Y * (1.1 * omega01 * r1.Y + 1.1 * v02x - 1.1 * v01x) + std::pow(r2.Y, 2) * (-1.1 * omega01 * r1.X + 1.1 * v02y - 0.1 * v01y) + std::pow(r2.X, 2) * v01y)) + tri.inertia * (inertia * std::pow(tri.mass, 2) * (-1.1 * omega01 * r1.X + 1.1 * omega02 * r2.X + 1.1 * v02y - 0.1 * v01y) + std::pow(mass, 2) * (inertia + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.mass) * v01y + mass * tri.mass * (inertia * (-1.1 * omega01 * r1.X + 1.1 * omega02 * r2.X + 1.1 * v02y + 0.9 * v01y) + tri.mass * (r1.X * r1.Y * (-1.1 * omega02 * r2.Y + 1.1 * v02x - 1.1 * v01x) + std::pow(r1.Y, 2) * (1.1 * omega02 * r2.X + 1.1 * v02y - 0.1 * v01y) + std::pow(r1.X, 2) * v01y)))) / (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2))));
	vel2.X = (mass * tri.mass * (mass * (1. * std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + 1. * std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass * v02x + (1. * std::pow(r1.X, 2) + 1. * std::pow(r1.Y, 2)) * tri.inertia * tri.mass * v02x + mass * tri.inertia * (1. * std::pow(r1.Y, 2) * v02x + std::pow(r1.X, 2) * (1.1 * omega02 * r2.Y - 0.1 * v02x + 1.1 * v01x) + r1.X * r1.Y * (-1.1 * omega02 * r2.X - 1.1 * v02y + 1.1 * v01y))) + inertia * (tri.inertia * (1. * std::pow(tri.mass, 2) * v02x + std::pow(mass, 2) * (-1.1 * omega01 * r1.Y + 1.1 * omega02 * r2.Y - 0.1 * v02x + 1.1 * v01x) + mass * tri.mass * (-1.1 * omega01 * r1.Y + 1.1 * omega02 * r2.Y + 0.9 * v02x + 1.1 * v01x)) + mass * tri.mass * ((1. * std::pow(r2.X, 2) + 1. * std::pow(r2.Y, 2)) * tri.mass * v02x + mass * (1. * std::pow(r2.Y, 2) * v02x + std::pow(r2.X, 2) * (-1.1 * omega01 * r1.Y - 0.1 * v02x + 1.1 * v01x) + r2.X * r2.Y * (1.1 * omega01 * r1.X - 1.1 * v02y + 1.1 * v01y))))) / (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2))));
	vel2.Y = (mass * tri.mass * (inertia * (1. * std::pow(r2.X, 2) + 1. * std::pow(r2.Y, 2)) * tri.mass * v02y + mass * (1. * std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + 1. * std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass * v02y + inertia * mass * (1. * std::pow(r2.X, 2) * v02y + r2.X * r2.Y * (-1.1 * omega01 * r1.Y - 1.1 * v02x + 1.1 * v01x) + std::pow(r2.Y, 2) * (1.1 * omega01 * r1.X - 0.1 * v02y + 1.1 * v01y))) + tri.inertia * (1. * inertia * std::pow(tri.mass, 2) * v02y + mass * tri.mass * ((1. * std::pow(r1.X, 2) + 1. * std::pow(r1.Y, 2)) * tri.mass * v02y + inertia * (1.1 * omega01 * r1.X - 1.1 * omega02 * r2.X + 0.9 * v02y + 1.1 * v01y)) + std::pow(mass, 2) * (inertia * (1.1 * omega01 * r1.X - 1.1 * omega02 * r2.X - 0.1 * v02y + 1.1 * v01y) + tri.mass * (1. * std::pow(r1.X, 2) * v02y + r1.X * r1.Y * (1.1 * omega02 * r2.Y - 1.1 * v02x + 1.1 * v01x) + std::pow(r1.Y, 2) * (-1.1 * omega02 * r2.X - 0.1 * v02y + 1.1 * v01y))))) / (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2))));
	omega1 = (tri.inertia * (inertia * omega01 * std::pow(tri.mass, 2) + std::pow(mass, 2) * (inertia * omega01 + tri.mass * (-0.1 * omega01 * std::pow(r1.Y, 2) + r1.Y * (1.1 * omega02 * r2.Y - 1.1 * v02x + 1.1 * v01x) + r1.X * (-0.1 * omega01 * r1.X + 1.1 * omega02 * r2.X + 1.1 * v02y - 1.1 * v01y))) + mass * tri.mass * (2. * inertia * omega01 + tri.mass * (-0.1 * omega01 * std::pow(r1.Y, 2) + r1.Y * (1.1 * omega02 * r2.Y - 1.1 * v02x + 1.1 * v01x) + r1.X * (-0.1 * omega01 * r1.X + 1.1 * omega02 * r2.X + 1.1 * v02y - 1.1 * v01y)))) + mass * tri.mass * (inertia * mass * omega01 * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) + inertia * omega01 * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass + mass * tri.mass * (-0.1 * omega01 * std::pow(r1.Y, 2) * std::pow(r2.X, 2) + r1.X * r2.Y * (-0.1 * omega01 * r1.X * r2.Y + 1.1 * r2.X * v02x + 1.1 * r2.Y * v02y - 1.1 * r2.X * v01x - 1.1 * r2.Y * v01y) + r1.Y * r2.X * (0.2 * omega01 * r1.X * r2.Y - 1.1 * r2.X * v02x - 1.1 * r2.Y * v02y + 1.1 * r2.X * v01x + 1.1 * r2.Y * v01y)))) / (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2))));
	omega2 = (std::pow(mass, 2) * tri.inertia * std::pow(tri.mass, 2) * (std::pow(r1.X, 2) * r2.Y * (-0.1 * omega02 * r2.Y + 0.1 * v02x - 0.1 * v01x) + std::pow(r1.Y, 2) * r2.X * (-0.1 * omega02 * r2.X - 0.1 * v02y + 0.1 * v01y) + r1.X * r1.Y * (0.1 * r2.Y * v02y + r2.X * (0.2 * omega02 * r2.Y - 0.1 * v02x + 0.1 * v01x) - 0.1 * r2.Y * v01y)) + std::pow(inertia, 2) * (omega02 * tri.inertia * std::pow(tri.mass, 2) + std::pow(mass, 2) * (omega02 * tri.inertia + r2.Y * tri.mass * (omega01 * r1.Y + v02x - 1. * v01x) + r2.X * tri.mass * (omega01 * r1.X - 1. * v02y + v01y)) + mass * tri.mass * (2. * omega02 * tri.inertia + r2.Y * tri.mass * (omega01 * r1.Y + v02x - 1. * v01x) + r2.X * tri.mass * (omega01 * r1.X - 1. * v02y + v01y))) + inertia * mass * tri.mass * (tri.inertia * tri.mass * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) - 0.1 * omega02 * std::pow(r2.X, 2) - 0.1 * omega02 * std::pow(r2.Y, 2) + r2.Y * (0.1 * omega01 * r1.Y + 0.1 * v02x - 0.1 * v01x) + r2.X * (0.1 * omega01 * r1.X - 0.1 * v02y + 0.1 * v01y)) + mass * (tri.inertia * (omega02 * (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) - 0.1 * omega02 * std::pow(r2.X, 2) - 0.1 * omega02 * std::pow(r2.Y, 2) + r2.Y * (0.1 * omega01 * r1.Y + 0.1 * v02x - 0.1 * v01x) + r2.X * (0.1 * omega01 * r1.X - 0.1 * v02y + 0.1 * v01y)) + tri.mass * (std::pow(r1.X, 2) * r2.Y * (v02x - 1. * v01x) + r1.X * r1.Y * (r2.X * (-1. * v02x + v01x) + r2.Y * (v02y - 1. * v01y)) + std::pow(r1.Y, 2) * r2.X * (-1. * v02y + v01y))))) / (inertia * (mass * tri.mass * (mass * (std::pow(r1.Y, 2) * std::pow(r2.X, 2) - 2. * r1.X * r1.Y * r2.X * r2.Y + std::pow(r1.X, 2) * std::pow(r2.Y, 2)) * tri.mass + (std::pow(r1.X, 2) + std::pow(r1.Y, 2)) * tri.inertia * (mass + tri.mass)) + inertia * (mass * (std::pow(r2.X, 2) + std::pow(r2.Y, 2)) * tri.mass * (mass + tri.mass) + tri.inertia * (std::pow(mass, 2) + 2. * mass * tri.mass + std::pow(tri.mass, 2)))));*/

	gsl_matrix* equations = gsl_matrix_alloc(8, 8);
	gsl_vector* yvector = gsl_vector_alloc(8);
	gsl_vector* xvector = gsl_vector_alloc(8);

	gsl_matrix_set_zero(equations);
	gsl_matrix_set(equations, 0, 0, mass); gsl_matrix_set(equations, 0, 6, -1);
	gsl_matrix_set(equations, 1, 1, mass); gsl_matrix_set(equations, 1, 7, -1);
	gsl_matrix_set(equations, 2, 2, tri.mass); gsl_matrix_set(equations, 2, 6, 1);
	gsl_matrix_set(equations, 3, 3, tri.mass); gsl_matrix_set(equations, 3, 7, 1);
	gsl_matrix_set(equations, 4, 4, -inertia); gsl_matrix_set(equations, 4, 6, -r1.Y); gsl_matrix_set(equations, 4, 7, r1.X);
	gsl_matrix_set(equations, 5, 5, -tri.inertia); gsl_matrix_set(equations, 5, 6, r2.Y); gsl_matrix_set(equations, 5, 7, -r2.X);
	gsl_matrix_set(equations, 6, 0, 1); gsl_matrix_set(equations, 6, 2, -1); gsl_matrix_set(equations, 6, 4, -r1.Y); gsl_matrix_set(equations, 6, 5, r2.Y);
	gsl_matrix_set(equations, 7, 1, 1); gsl_matrix_set(equations, 7, 3, -1); gsl_matrix_set(equations, 7, 4, r1.X); gsl_matrix_set(equations, 7, 5, -r2.X);

	gsl_vector_set(yvector, 0, mass * v01x); 
	gsl_vector_set(yvector, 1, mass * v01y);
	gsl_vector_set(yvector, 2, tri.mass * v02x);
	gsl_vector_set(yvector, 3, tri.mass * v02y);
	gsl_vector_set(yvector, 4, -inertia * omega01);
	gsl_vector_set(yvector, 5, -tri.inertia * omega02);
	gsl_vector_set(yvector, 6, 0);
	gsl_vector_set(yvector, 7, 0);

	gsl_vector* tau = gsl_vector_alloc(8);

	gsl_linalg_QR_decomp(equations, tau);
	gsl_linalg_QR_solve(equations, tau, yvector, xvector);

	PointF tmp;
	tmp.X = gsl_vector_get(xvector, 6);
	tmp.Y = gsl_vector_get(xvector, 7);

	if (inner_product(sur, tmp) > 0)
	{
		vel1.X = gsl_vector_get(xvector, 0);
		vel1.Y = gsl_vector_get(xvector, 1);
		vel2.X = gsl_vector_get(xvector, 2);
		vel2.Y = gsl_vector_get(xvector, 3);
		omega1 = gsl_vector_get(xvector, 4);
		omega2 = gsl_vector_get(xvector, 5);
	}

	gsl_matrix_free(equations);
	gsl_vector_free(xvector);
	gsl_vector_free(yvector);
	gsl_vector_free(tau);
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