#ifndef PROJEKT_RECTANGLE_H
#define PROJEKT_RECTANGLE_H

#include "UniversalConvexShape.h"

class ProjektRectangle : 
	public UniversalConvexShape
{

public:
	ProjektRectangle(int = 200, int = 200, float = 0, Vector2f = { 0, 0 });
};



#endif
