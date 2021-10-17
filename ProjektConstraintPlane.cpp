#include "ProjektConstraintPlane.h"


void ProjektConstraintPlane::activateImpulse()
{
	Intersection is{ ucs.collisionWithPlane(position, dir) };
	ProjektConstraintPlane::SubConstraint sc;
	if (is.collision)
	{
		sc.cp = is.point.index;
		sc.constraint_error = dir.dot(is.point - position);
		active = true;
		if (subconstraints.empty()) subconstraints.push_back(sc);
		else subconstraints[0] = sc;
	}
	else
		active = false;
}

float ProjektConstraintPlane::calcImpulse(float dt)
{
	if (active)
	{
		float impulse_squared = 0;
		for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
		{
			Vector2f r{ ucs.getTransformedVertex(sc.cp) };
			sc.jn = { dir.x(), dir.y(), r.x() * dir.y() - r.y() * dir.x() };
			sc.jt = { tan.x(), tan.y(), r.x() * tan.y() - r.y() * tan.x() };

			float a11, a12, a21, a22;
			a11 = sc.jn(0) * sc.jn(0) / ucs.mass + sc.jn(1) * sc.jn(1) / ucs.mass + sc.jn(2) * sc.jn(2) / ucs.inertia;
			a12 = sc.jn(0) * sc.jt(0) / ucs.mass + sc.jn(1) * sc.jt(1) / ucs.mass + sc.jn(2) * sc.jt(2) / ucs.inertia;
			a21 = a12;
			a22 = sc.jt(0) * sc.jt(0) / ucs.mass + sc.jt(1) * sc.jt(1) / ucs.mass + sc.jt(2) * sc.jt(2) / ucs.inertia;

			float inv_det = 1 / (a11 * a22 - a12 * a21);
			float cn = sc.jn(0) * ucs.vel(0) + sc.jn(1) * ucs.vel(1) + sc.jn(2) * ucs.omega + sc.constraint_error / dt * beta;
			float ct = sc.jt(0) * ucs.vel(0) + sc.jt(1) * ucs.vel(1) + sc.jt(2) * ucs.omega;

			sc.ln = -inv_det * (a22 * cn - a21 * ct);
			sc.lt = -inv_det * (-a12 * cn + a11 * ct);
			//impulse clamping
			if (sc.accln + sc.ln > 0)
				sc.ln = -sc.accln;
#ifdef FRICTION_ON
			if (2 * abs(sc.accln + sc.ln) * friction_coeff < abs(sc.acclt + sc.lt))
			{
				if (sc.acclt + sc.lt > 0)
					sc.lt = abs(sc.accln) * friction_coeff - sc.acclt;
				else
					sc.lt = -abs(sc.accln) * friction_coeff - sc.acclt;
			}
#endif // FRICTION_ON
			
			sc.accln += sc.ln;
			sc.acclt += sc.lt;

			impulse_squared += sc.lt * sc.lt + sc.ln * sc.ln;
		}
		return impulse_squared;
	}
	return 0.0f;
}





void ProjektConstraintPlane::applyImpulse()
{
	if (active)
	{
		for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
		{
			ucs.vel += (sc.ln * Vector2f{ sc.jn(0), sc.jn(1) } + sc.lt * Vector2f{ sc.jt(0), sc.jt(1) } ) / ucs.mass;
			ucs.omega += (sc.ln * sc.jn(2) + sc.lt * sc.jt(2)) / ucs.inertia;
		}
	}
}

void ProjektConstraintPlane::storeAccImpulse() 
{
}
	

void ProjektConstraintPlane::applyAccImpulse() 
{
	for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
	{
		sc.accln = 0;
		sc.acclt = 0;
	}
}


