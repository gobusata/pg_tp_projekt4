#pragma once
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
class ProjektConstraintPlane:
	public ProjektConstraint
{
private:
	UniversalConvexShape & ucs;
	const Vector2f position, dir, tan;
	Vector3f jt, jn;
	bool active;
	float lt = 0, ln = 0, acclt = 0, accln = 0;
	float constraint_error;
	const float beta = 0.05, friction_coeff = 0.1;
	Vector2f touching_point;
	Vector2f tmp_r;
public:
	ProjektConstraintPlane(UniversalConvexShape& _ucs, Vector2f _dir, Vector2f _pos) :
		ucs(_ucs), dir(_dir), tan({ cross(dir, 1) }), position(_pos) {};

	void activateImpulse() override;
	
	float calcImpulse(float dt) override;
	
	void applyImpulse() override;
	
	void storeAccImpulse() override;
	
	void applyAccImpulse() override;
};

