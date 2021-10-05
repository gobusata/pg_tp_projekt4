#ifndef PROJEKT_RECTANGLE_H
#define PROJEKT_RECTANGLE_H

#include "UniversalConvexShape.h"

class ProjektRectangle : 
	public UniversalConvexShape
{

public:
	ProjektRectangle(int x = 200, int y = 200, float r = 0, Vector2f v = { 0, 0 }, float scale = 1.0);
};



#endif
