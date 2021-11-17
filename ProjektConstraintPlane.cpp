#include "ProjektConstraintPlane.h"

float ProjektConstraintPlane::borderZoneWidth = 2.0f;
float ProjektConstraintPlane::beta = 0.00;
float ProjektConstraintPlane::friction_coeff = 0.5;
bool ProjektConstraintPlane::warm_start = false;

using SubConstraint = ProjektConstraintPlane::SubConstraint;


void ProjektConstraintPlane::activateImpulse()
{
	Rotation2Df trans1{ Rotation2Df(ucs.pos(2)) };	
	std::vector<float> t1(ucs.vertices.size());
	std::transform(ucs.vertices.begin(), ucs.vertices.end(), t1.begin(),
		[this, &trans1](Vector2f v)->float
		{
			return dir.dot(position - Vector2f{ ucs.pos(0), ucs.pos(1) } - trans1 * v) - borderZoneWidth;
		}
	);

	int i1 = -1, i2 = -1;
	for (int i = 0; i < t1.size(); i++)
	{
		if (t1[i] < 0 && t1[(i + 1) % t1.size()] > 0)
			i1 = i;
		if (t1[i] > 0 && t1[(i + 1) % t1.size()] < 0)
			i2 = (i + 1) % t1.size();
	}
	if (i1 == i2) i2 = -1;

	std::vector<std::pair<int, float>> prev_acc_lambda(subconstraints.size());
	std::transform(subconstraints.begin(), subconstraints.end(), prev_acc_lambda.begin(),
		[](const SubConstraint& sc)->std::pair<int, float> {return { sc.cp, sc.accln }; });
	subconstraints.clear();
		
	if (i1 != -1)
	{
		std::vector<std::pair<int, float>>::iterator pali = std::find_if(prev_acc_lambda.begin(), prev_acc_lambda.end(),
			[i1](const std::pair<int, float>& sci)->bool {return sci.first == i1; });
		subconstraints.push_back(SubConstraint(i1));
		if (pali != prev_acc_lambda.end())
			subconstraints.back().accln = pali->second;
	}

	if (i2 != -1)
	{
		std::vector<std::pair<int, float>>::iterator pali = std::find_if(prev_acc_lambda.begin(), prev_acc_lambda.end(),
			[i2](const std::pair<int, float>& sci)->bool {return sci.first == i2; });
		subconstraints.push_back(SubConstraint(i2));
		if (pali != prev_acc_lambda.end())
			subconstraints.back().accln = pali->second;
	}
		
	if (!subconstraints.empty())
		active = true;
	else active = false;
}


float ProjektConstraintPlane::calcImpulse(SubConstraint & sc, float dt)
{
	float error = 0;
	if (active)
	{
		if (sc.active)
		{
			Vector2f r{ ucs.getTransformedVertex(sc.cp) };
			sc.jn = { dir.x(), dir.y(), r.x() * dir.y() - r.y() * dir.x() };
			sc.ln = -1 / (sc.jn(0) * sc.jn(0) / ucs.mass + sc.jn(1) * sc.jn(1) / ucs.mass + sc.jn(2) * sc.jn(2) / ucs.inertia) *
				(sc.jn(0) * ucs.vel(0) + sc.jn(1) * ucs.vel(1) + sc.jn(2) * ucs.vel(2) + sc.constraint_error / dt * beta);
			if (sc.accln + sc.ln > 0)
				sc.ln = -sc.accln;
			sc.accln += sc.ln;

			sc.jt = { tan.x(), tan.y(), r.x() * tan.y() - r.y() * tan.x() };
			sc.lt = -1 / (sc.jt(0) * sc.jt(0) / ucs.mass + sc.jt(1) * sc.jt(1) / ucs.mass + sc.jt(2) * sc.jt(2) / ucs.inertia) *
				(sc.jt(0) * ucs.vel(0) + sc.jt(1) * ucs.vel(1) + sc.jt(2) * ucs.vel(2));
				
			if (sc.accln >= 0)
				sc.lt = -sc.acclt;
			sc.lt = 0;
			sc.acclt += sc.lt;

			if (sc.accln != 0)
				error = std::max(error, abs(sc.ln / sc.accln));
			if (sc.acclt != 0)
				error = std::max(error, abs(sc.lt / sc.acclt));
		}
	}
	return error;
}


void ProjektConstraintPlane::applyImpulse(SubConstraint & sc)
{
	if (active)
	{
		if (sc.active)
		{
			ucs.vel(0) += (sc.ln * sc.jn(0) + sc.lt * sc.jt(0)) / ucs.mass;
			ucs.vel(1) += (sc.ln * sc.jn(1) + sc.lt * sc.jt(1)) / ucs.mass;
			ucs.vel(2) += (sc.ln * sc.jn(2) + sc.lt * sc.jt(2)) / ucs.inertia;
		}
	}
}

float ProjektConstraintPlane::calcApplyImpulse(float dt)
{
	float error = 0;
	for (SubConstraint& sc : subconstraints)
	{
		error = std::max(calcImpulse(sc, dt), error);
		applyImpulse(sc);
	}
	return error;
}


void ProjektConstraintPlane::storeAccImpulse() 
{
	if (active)
	{
		for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
		{
			if (sc.active)
			{
				float c = sc.jn(0) * ucs.vel(0) + sc.jn(1) * ucs.vel(1) + sc.jn(2) * ucs.vel(2);
				//dbgmsg("ProjektConstraintPlane: constraint address = {} subconstraint = {}, c = {}",
				//	static_cast<void*>(this), static_cast<void*>(&sc), c);
			}
		}
	}
}


float ProjektConstraintPlane::applyAccImpulse() 
{
	for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
	{
		if (sc.active)
		{
			if (warm_start)
			{
				dbgmsg("sc.cp = {:.2f} sc.jn = {:.2f}, subconstraints.size() = {}", 
					sc.cp, sc.jn, subconstraints.size());
				ucs.vel(0) += (sc.accln * sc.jn(0) + sc.acclt * sc.jt(0)) / ucs.mass;
				ucs.vel(1) += (sc.accln * sc.jn(1) + sc.acclt * sc.jt(1)) / ucs.mass;
				ucs.vel(2) += (sc.accln * sc.jn(2) + sc.acclt * sc.jt(2)) / ucs.inertia;
			}
			else
			{
				sc.accln = 0;
				sc.acclt = 0;
			}
		}
	}
	return 1;
}


Vector2f ProjektConstraintPlane::pointOfContact(const ProjektConstraintPlane::SubConstraint& sc) const
{
	return ucs.getVertexPos(sc.cp);
}

