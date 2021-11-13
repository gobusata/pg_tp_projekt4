#include "ProjektConstraintPlane.h"

float ProjektConstraintPlane::borderZoneWidth = 5;
float ProjektConstraintPlane::beta = 0.00;
float ProjektConstraintPlane::friction_coeff = 0.5;

using SubConstraint = ProjektConstraintPlane::SubConstraint;

void ProjektConstraintPlane::activateImpulse()
{
	Rotation2Df trans1{ Rotation2Df(ucs.rot) };	
	std::vector<float> t1(ucs.vertices.size());
	std::transform(ucs.vertices.begin(), ucs.vertices.end(), t1.begin(),
		[this, &trans1](Vector2f v)->float
		{
			return dir.dot(position - ucs.pos - trans1 * v) - borderZoneWidth;
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

float ProjektConstraintPlane::calcImpulse(float dt)
{
	if (active)
	{
		float impulse_squared = 0;
		for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
		{
			if (sc.active)
			{
				Vector2f r{ ucs.getTransformedVertex(sc.cp) };
				sc.jn = { dir.x(), dir.y(), r.x() * dir.y() - r.y() * dir.x() };
				sc.ln = -1 / (sc.jn(0) * sc.jn(0) / ucs.mass + sc.jn(1) * sc.jn(1) / ucs.mass + sc.jn(2) * sc.jn(2) / ucs.inertia) *
					(sc.jn(0) * ucs.vel(0) + sc.jn(1) * ucs.vel(1) + sc.jn(2) * ucs.omega + sc.constraint_error / dt * beta);
				if (sc.accln + sc.ln > 0)
					sc.ln = -sc.accln;
				sc.accln += sc.ln;

				sc.jt = { tan.x(), tan.y(), r.x() * tan.y() - r.y() * tan.x() };
				sc.lt = -1 / (sc.jt(0) * sc.jt(0) / ucs.mass + sc.jt(1) * sc.jt(1) / ucs.mass + sc.jt(2) * sc.jt(2) / ucs.inertia) *
					(sc.jt(0) * ucs.vel(0) + sc.jt(1) * ucs.vel(1) + sc.jt(2) * ucs.omega);
				
				if (sc.accln >= 0)
					sc.lt = -sc.acclt;
				sc.lt = 0;
				sc.acclt += sc.lt;

				impulse_squared += sc.ln * sc.ln + sc.lt * sc.lt;
			}
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
			if (sc.active)
			{
				ucs.vel += (sc.ln * Vector2f{ sc.jn(0), sc.jn(1) } + sc.lt * Vector2f{ sc.jt(0), sc.jt(1) }) / ucs.mass;
				ucs.omega += (sc.ln * sc.jn(2) + sc.lt * sc.jt(2)) / ucs.inertia;
			}
		}
	}
}

void ProjektConstraintPlane::storeAccImpulse() 
{
	if (active)
	{
		for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
		{
			if (sc.active)
			{
				0;
			}
		}
	}
}
	

float ProjektConstraintPlane::applyAccImpulse() 
{
	float ret = 0;
	for (ProjektConstraintPlane::SubConstraint& sc : subconstraints)
	{
		if (sc.active)
		{
			dbgmsg("sc.cp = {:.2f} sc.jn = {:.2f}, subconstraints.size() = {}", 
				sc.cp, sc.jn, subconstraints.size());
			//warm start
			ucs.vel += (sc.accln * Vector2f{ sc.jn(0), sc.jn(1) } + sc.acclt * Vector2f{ sc.jt(0), sc.jt(1) }) / ucs.mass;
			ucs.omega += (sc.accln * sc.jn(2) + sc.acclt * sc.jt(2)) / ucs.inertia;
			ret += sc.accln * sc.accln + sc.acclt * sc.acclt;
		}
		// reseting accumulated impulse from previous timepoint
		//sc.accln = 0;
		//sc.acclt = 0;	
	}

	return ret;
}

Vector2f ProjektConstraintPlane::point_of_contact(const ProjektConstraintPlane::SubConstraint& sc) const
{
	return ucs.getVertexPos(sc.cp);
}

