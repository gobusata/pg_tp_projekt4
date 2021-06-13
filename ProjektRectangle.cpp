#include "ProjektRectangle.h"

ProjektRectangle::ProjektRectangle(int x, int y, float r, Vector2f v): UniversalConvexShape()
{
	
	float width = 20, height = 20;
	vertices.resize(4);
		
	vertices[0] << width , height;
	vertices[1] << -width , height;
	vertices[2] << -width , -height;
	vertices[3] << width, -height;

	pos << x, y;
	rot = r;
	vel = v;
	omega = 0.000;
	createShape();	
}



