#pragma once
#include <stdexcept>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"
class ProjektConstraintNoPenetration :
    public ProjektTwoBodyConstraint
{
public:
    struct SubConstraint;
    std::vector<SubConstraint> subconstraints;
    GjkSimplex intersection;
private:
    static float friction_coeff, beta, borderZoneWidth;
    bool shapes_in_close_proximity;
    float getDistance(const ProjektConstraintNoPenetration::SubConstraint& sc);

public:

    ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b);

    Vector2f pointOfContact(const ProjektConstraintNoPenetration::SubConstraint& sc) const;

    enum TouchingFeature
    {
        vertex, edge
    };

    class SubConstraint
    {
    public:
        Vector2f normal{ 0, 0 };
        bool active{ true };
        float ln{ 0 }, lt{ 0 }, acc_ln{ 0 }, acc_lt{ 0 }, constraint_error{ 0 };
        int  cpa[2], cpb[2];
        TouchingFeature tfa, tfb;
        Eigen::Matrix<float, 1, 6> jmat;
        SubConstraint() :
            constraint_error{ 0 }, active{ false } {}
        void getTf(const GjkSimplex& _i, const ClosestFeature& _cs);
    };
    
    bool eq(const ProjektConstraintNoPenetration::SubConstraint& a, const ProjektConstraintNoPenetration::SubConstraint& b);

    void activateImpulse() override;

    float calcApplyImpulse(float dt) override;
    
    void storeAccImpulse() override;
    
    float applyAccImpulse() override;

    float calcImpulse(SubConstraint & sc, float dt);

    void applyImpulse(SubConstraint & sc);
};

bool operator==(const ProjektConstraintNoPenetration::SubConstraint& a,
    const ProjektConstraintNoPenetration::SubConstraint& b);