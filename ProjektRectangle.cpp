#include "ProjektRectangle.h"

ProjektRectangle::ProjektRectangle(int x, int y, float r, Vector2f v, float scale): UniversalConvexShape()
{
	
	float width = scale*20, height = scale*20;
	vertices.resize(4);
		
	vertices[0] << width , height;
	vertices[1] << -width , height;
	vertices[2] << -width , -height;
	vertices[3] << width, -height;

	pos << x, y;
	rot = r;
	vel = v;
	omega = 0.000;
	inertia = mass / 3 * (width * width + height * height);
	createShape();	
}





