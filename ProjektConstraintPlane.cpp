#include "ProjektConstraintPlane.h"

float ProjektConstraintPlane::borderZoneWidth = 0.f;
float ProjektConstraintPlane::beta = 0.05;
float ProjektConstraintPlane::friction_coeff = 0.02;

using SubConstraint = ProjektConstraintPlane::SubConstraint;


void ProjektConstraintPlane::activateImpulse()
{
	Rotation2Df trans1{ Rotation2Df(ucs.pos(2)) };	
	std::vector<float> t1(ucs.pcp.vs.size());
	std::transform(ucs.pcp.vs.begin(), ucs.pcp.vs.end(), t1.begin(),
		[this, &trans1](Vector2f v)->float
		{
			return dir.dot(Vector2f{ ucs.pos(0), ucs.pos(1) } + trans1 * v - position) - borderZoneWidth;
		}
	);

	int i1 = -1, i2 = -1;
	for (int i = 0; i < t1.size(); i++)
	{
		if (t1[i] < 0 && t1[(i + 1) % t1.size()] > 0)
			i1 = (i+1) % t1.size();
		if (t1[i] > 0 && t1[(i + 1) % t1.size()] < 0)
			i2 = i;
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
		subconstraints.back().constraint_error = t1[i1];
		if (pali != prev_acc_lambda.end())
			subconstraints.back().accln = pali->second;
	}

	if (i2 != -1)
	{
		std::vector<std::pair<int, float>>::iterator pali = std::find_if(prev_acc_lambda.begin(), prev_acc_lambda.end(),
			[i2](const std::pair<int, float>& sci)->bool {return sci.first == i2; });
		subconstraints.push_back(SubConstraint(i2));
		subconstraints.back().constraint_error = t1[i2];
		if (pali != prev_acc_lambda.end())
			subconstraints.back().accln = pali->second;
	}
		
	if (!subconstraints.empty())
		active = true;
	else active = false;
}

float ProjektConstraintPlane::calcApplyImpulse(float dt)
{
	float err = 0;
	for (std::vector<SubConstraint>::iterator sci = subconstraints.begin(); sci != subconstraints.end(); sci++)
	{
		err = max(err, calcImpulse(*sci, dt));
		applyImpulse(*sci);
	}
	return err;
}


float ProjektConstraintPlane::calcImpulse(SubConstraint & sc, float dt)
{
	float error = 0;
	if (active)
	{
		if (sc.active)
		{
			Vector2f r{ ucs.pcp.getTransformedVertex(sc.cp) };
			sc.jn = { dir.x(), dir.y(), r.x() * dir.y() - r.y() * dir.x() };
			sc.ln = -1 / (sc.jn.transpose() * ucs.kmat * sc.jn) * (sc.jn.dot(ucs.vel) + sc.constraint_error / dt * beta);
			if (sc.accln + sc.ln > 0)
				sc.ln = -sc.accln;
			sc.accln += sc.ln;

			sc.jt = { tan.x(), tan.y(), r.x() * tan.y() - r.y() * tan.x() };
			sc.lt = -1 / (sc.jt.transpose() * ucs.kmat * sc.jt) * sc.jt.dot(ucs.vel);
		
			float nimp = friction_coeff * abs(sc.accln);
			if (nimp < sc.acclt + sc.lt)
				sc.lt = nimp - sc.acclt;
			else if (-nimp > sc.acclt + sc.lt)
				sc.lt = -nimp - sc.acclt;

			sc.acclt += sc.lt;

			if (sc.accln != 0)
				error = (sc.accln != 0.f)? max(error, abs(sc.ln / sc.accln)):error;
			if (sc.acclt != 0)
				error = (sc.acclt != 0.f)? max(error, abs(sc.lt / sc.acclt)):error;
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
			ucs.vel += ucs.kmat * sc.jn * sc.ln + ucs.kmat * sc.jt * sc.lt;
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
				ucs.vel += ucs.kmat * (sc.accln * sc.jn + sc.acclt * sc.jt);
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
	return ucs.pcp.getVertexPos(sc.cp);
}

