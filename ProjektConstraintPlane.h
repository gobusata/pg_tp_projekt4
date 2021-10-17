#pragma once
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
#define FRICTION_OFF
class ProjektConstraintPlane:
	public ProjektConstraint
{
public:
	struct SubConstraint;
private:
	UniversalConvexShape & ucs;
	const Vector2f position, dir, tan;
	bool active;
	const float beta = 0.02, friction_coeff = 0.05;
	Vector2f tmp_r;
	std::vector<SubConstraint> subconstraints;
public:
	ProjektConstraintPlane(UniversalConvexShape& _ucs, Vector2f _dir, Vector2f _pos) :
		ucs(_ucs), dir(_dir), tan({ cross(dir, 1) }), position(_pos) {};

	struct SubConstraint
	{
		float ln, lt, accln, acclt, constraint_error;
		int cp;
		Vector3f jt, jn;
	};

	void activateImpulse() override;
	
	float calcImpulse(float dt) override;
	
	void applyImpulse() override;
	
	void storeAccImpulse() override;
	
	void applyAccImpulse() override;
};

