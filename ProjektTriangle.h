#ifndef PROJEKT_TRIANGLE_H
#define PROJEKT_TRIANGLE_H

#include "UniversalConvexShape.h"


class ProjektTriangle :
    public UniversalConvexShape
{
public:
    ProjektTriangle(int = 200, int = 200, float = 0, Vector2f = { 0, 0 });

};

#endif
