#ifndef PROJEKT_CONSTRAINT_H
#define PROJEKT_CONSTRAINT_H
#include <algorithm>

class ProjektConstraint
{
public:
	virtual void activateImpulse() = 0;
	virtual float calcApplyImpulse(float dt) = 0;
	virtual void storeAccImpulse() = 0;
	virtual float applyAccImpulse() = 0;
};
#endif

