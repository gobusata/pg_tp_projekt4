#ifndef PROJEKT_CONSTRAINT_H
#define PROJEKT_CONSTRAINT_H
#include <algorithm>
#include "UniversalConvexShape.h"

class ProjektConstraint
{
public:
	static bool warm_start;
	struct SubConstraint;
	virtual void activateImpulse() = 0;
	virtual float calcApplyImpulse(float dt) = 0;
	virtual void storeAccImpulse() = 0;
	virtual float applyAccImpulse() = 0;
};

class ProjektSubConstraint
{
public:
	virtual float calcImpulse(float dt) = 0;
	virtual void applyImpulse() = 0;
};

class ProjektTwoBodyConstraint :public ProjektConstraint
{
public:
	ProjektConvexBody &a, &b;
	DiagonalMatrix<float, 6> kmat;
	ProjektTwoBodyConstraint(ProjektConvexBody& _a, ProjektConvexBody& _b);
};
#endif

