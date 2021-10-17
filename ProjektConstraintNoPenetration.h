#pragma once
#include <stdexcept>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
class ProjektConstraintNoPenetration :
    public ProjektConstraint
{
public:
    struct SubConstraint;
private:
    
    
    UniversalConvexShape &a, &b;
    MatrixXf minv_mat;
    const float friction_coeff = 0.2, beta = 5;
    bool shapes_in_close_proximity, active;
    std::vector<SubConstraint> subconstraints;
    Intersection intersection;

    float get_distance(const ProjektConstraintNoPenetration::SubConstraint& sc);

public:

    ProjektConstraintNoPenetration(UniversalConvexShape& _a, UniversalConvexShape & _b);

    Vector2f point_of_contact(const ProjektConstraintNoPenetration::SubConstraint& sc);

    

    enum TouchingFeature
    {
        vertex, edge
    };

    struct SubConstraint
    {
        Vector2f normal;
        bool active, valid;
        float ln, lt, acc_ln, acc_lt, constraint_error;
        int  cpa[2], cpb[2];
        TouchingFeature tfa, tfb;
        MatrixXf j_mat;
        SubConstraint() :
            j_mat(2, 6), constraint_error{ 0 }, valid{ false }, active{ false } {};
    };
    
    bool eq(const ProjektConstraintNoPenetration::SubConstraint& a, const ProjektConstraintNoPenetration& b);

    void activateImpulse() override;

    float calcImpulse(float dt) override;
    
    void applyImpulse() override;
    
    void storeAccImpulse() override;
    
    void applyAccImpulse() override;
};

bool operator==(const ProjektConstraintNoPenetration::SubConstraint& a,
    const ProjektConstraintNoPenetration::SubConstraint& b);