#pragma once
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
#define FRICTION_OFF
class ProjektConstraintPlane:
	public ProjektConstraint
{
private:
	ProjektConvexBody & ucs;
	const Vector2f position, dir, tan;
	bool active;
	Vector2f tmp_r;
public:
	struct SubConstraint;
	static float beta;
	static float borderZoneWidth, friction_coeff;
	std::vector<SubConstraint> subconstraints;

	ProjektConstraintPlane(ProjektConvexBody& _ucs, const Vector2f& _dir, const Vector2f& _pos) :
		ucs(_ucs), dir(_dir.normalized()), tan({ cross(dir, 1) }), position(_pos), active{ false } {};

	struct SubConstraint
	{
		float ln, lt, accln, acclt, constraint_error;
		int cp;
		bool active;
		Vector3f jt, jn;
		SubConstraint(int cp_) :
			ln{ 0 }, lt{ 0 }, jn{ 0, 0, 0 }, jt{ 0, 0, 0 },
			accln{ 0 }, acclt{ 0 }, constraint_error{ 0 }, cp{ cp_ }, active{ true } {};
	};

	void activateImpulse() override;
	
	float calcApplyImpulse(float dt) override;
	
	void storeAccImpulse() override;
	
	float applyAccImpulse() override;

	float calcImpulse(SubConstraint& sc, float dt);

	void applyImpulse(SubConstraint& sc);

	Vector2f pointOfContact(const ProjektConstraintPlane::SubConstraint& sc) const;
};

