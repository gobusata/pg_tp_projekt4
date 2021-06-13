#include "ProjektTriangle.h"

ProjektTriangle::ProjektTriangle(int x, int y, float r, Vector2f v) : UniversalConvexShape()
{

	float width = 60, height = 50;
	vertices.resize(3);

	vertices[0] << 0, -0.66*height;
	vertices[1] << 0.5*width, 0.33*height;
	vertices[2] << -0.5*width, 0.33*height;

	pos << x, y;
	vel = v;
	rot = r;
	omega = 0.000;
	createShape();

}
