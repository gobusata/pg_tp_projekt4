#pragma once
#include <stdexcept>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
class ProjektConstraintNoPenetration :
    public ProjektConstraint
{
public:
    struct SubConstraint;
    std::vector<SubConstraint> subconstraints;
private:
    
    
    UniversalConvexShape &a, &b;
    MatrixXf minv_mat;
    static float friction_coeff, beta, borderZoneWidth;
    static bool warm_start;
    bool shapes_in_close_proximity, active;
    Intersection intersection;
    float getDistance(const ProjektConstraintNoPenetration::SubConstraint& sc);

public:

    ProjektConstraintNoPenetration(UniversalConvexShape& _a, UniversalConvexShape & _b);

    Vector2f pointOfContact(const ProjektConstraintNoPenetration::SubConstraint& sc) const;

    

    enum TouchingFeature
    {
        vertex, edge
    };

    struct SubConstraint
    {
        Vector2f normal{ 0, 0 };
        bool active{ true }, valid{ true };
        float ln{ 0 }, lt{ 0 }, acc_ln{ 0 }, acc_lt{ 0 }, constraint_error{ 0 };
        int  cpa[2], cpb[2];
        TouchingFeature tfa, tfb;
        MatrixXf j_mat;
        SubConstraint() :
            j_mat(1, 6), constraint_error{ 0 }, valid{ false }, active{ false } {};
    };
    
    bool eq(const ProjektConstraintNoPenetration::SubConstraint& a, const ProjektConstraintNoPenetration& b);

    void activateImpulse() override;

    float calcApplyImpulse(float dt) override;
    
    void storeAccImpulse() override;
    
    float applyAccImpulse() override;

    float calcImpulse(SubConstraint & sc, float dt);

    void applyImpulse(SubConstraint & sc);
};

bool operator==(const ProjektConstraintNoPenetration::SubConstraint& a,
    const ProjektConstraintNoPenetration::SubConstraint& b);