#pragma once
#include <stdexcept>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
class ProjektConstraintNoPenetration :
    public ProjektConstraint
{
private:
    UniversalConvexShape &a, &b;
    const float friction_coeff = 0.2, beta = 0.32;
    float ln, lt;
    MatrixXf j_mat, minv_mat;
    bool shapes_in_close_proximity, active;
    float constraint_error;
    Intersection intersection;
    enum{ucs_a, ucs_b, ucs_ab} touching_shape;
    int touching_vertex[2];
    Vector2f normal;

public:
    inline Vector2f point_of_contact();
    ProjektConstraintNoPenetration(UniversalConvexShape& _a, UniversalConvexShape & _b);
    
    void activateImpulse() override;

    float calcImpulse(float dt) override;
    
    void applyImpulse() override;
    
    void storeAccImpulse() override;
    
    void applyAccImpulse() override;
};

