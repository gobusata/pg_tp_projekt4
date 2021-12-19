#include "ProjektConstraint.h"

bool ProjektConstraint::warm_start = false;

ProjektTwoBodyConstraint::ProjektTwoBodyConstraint(ProjektConvexBody& _a, ProjektConvexBody& _b) :
    a(_a), b(_b)
{
    for (int i = 0; i < 3; i++)
    {
        kmat.diagonal()[i] = _a.kmat.diagonal()[i];
        kmat.diagonal()[i + 3] = _b.kmat.diagonal()[i];
    }
}