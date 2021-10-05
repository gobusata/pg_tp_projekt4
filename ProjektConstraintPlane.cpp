#include "ProjektConstraintPlane.h"


void ProjektConstraintPlane::activateImpulse()
{
	Intersection is{ ucs.collisionWithPlane(position, dir) };
	if (is.collision)
	{
		touching_point = is.point;
		constraint_error = dir.dot(is.point - position);
		active = true;
	}
	else
		active = false;
}

float ProjektConstraintPlane::calcImpulse(float dt)
{

	if (active)
	{
		Vector2f r = touching_point - ucs.pos;
		jn = { dir.x(), dir.y(), r.x() * dir.y() - r.y() * dir.x() };
		jt = { tan.x(), tan.y(), r.x() * tan.y() - r.y() * tan.x() };

		float a11, a12, a21, a22;
		a11 = jn(0) * jn(0) / ucs.mass + jn(1) * jn(1) / ucs.mass + jn(2) * jn(2) / ucs.inertia;
		a12 = jn(0) * jt(0) / ucs.mass + jn(1) * jt(1) / ucs.mass + jn(2) * jt(2) / ucs.inertia;
		a21 = a12;
		a22 = jt(0) * jt(0) / ucs.mass + jt(1) * jt(1) / ucs.mass + jt(2) * jt(2) / ucs.inertia;

		float inv_det = 1 / (a11 * a22 - a12 * a21);
		float cn = jn(0) * ucs.vel(0) + jn(1) * ucs.vel(1) + jn(2) * ucs.omega + constraint_error / dt * beta;
		float ct = jt(0) * ucs.vel(0) + jt(1) * ucs.vel(1) + jt(2) * ucs.omega;

		ln = -inv_det * (a22 * cn - a21 * ct);
		lt = -inv_det * (-a12 * cn + a11 * ct);

		if (ln <= 0)
		{
			if (2 * abs(ln) * friction_coeff < abs(lt))
			{
				ln = -1 / (-a12 * friction_coeff * sgn(lt) + a11) * cn;
				lt = friction_coeff * abs(ln) * sgn(lt);
			}
		}
		return lt * lt + ln * ln;
	}
	return 0.0f;
}





void ProjektConstraintPlane::applyImpulse()
{
	if (active)
	{
		if (ln <= 0)
		{
			ucs.vel += ln * Vector2f{ jn(0), jn(1) } / ucs.mass + lt * Vector2f{ jt(0), jt(1) } / ucs.mass;
			ucs.omega += ln * jn(2)/ucs.inertia + lt * jt(2)/ucs.inertia;
		}	
	}
}

void ProjektConstraintPlane::storeAccImpulse() {}
void ProjektConstraintPlane::applyAccImpulse() {}


