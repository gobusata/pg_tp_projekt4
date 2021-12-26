#pragma once
#include <stdexcept>
#include <optional>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"


class ProjektConstraintNoPenetration :
    public ProjektTwoBodyConstraint
{
public:
    struct SubConstraint;
    std::vector<SubConstraint> subconstraints;
    GjkSimplex intersection;
    bool active;
private:
    static float friction_coeff, beta, borderZoneWidth, pointWidth;
    bool shapes_in_close_proximity;
    float getDistance(const ProjektConstraintNoPenetration::SubConstraint& sc);

public:

    ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b);

    Vector2f pointOfContact(const ProjektConstraintNoPenetration::SubConstraint& sc) const;

    class SubConstraint
    {
    public:
        Vector2f normal{ 0, 0 };
        bool active, valid, inBounds;
        float ln{ 0 }, lt{ 0 }, acc_ln{ 0 }, acc_lt{ 0 }, constraint_error{ 0 };
        Eigen::Matrix<float, 1, 6> jmat;
        SubConstraint(bool _valid = false, bool _inBounds = false, bool _active = false, float _constraint_error = 0.0f):
            active{ _active }, valid{ _valid }, inBounds{ _inBounds }, constraint_error{ _constraint_error } {}
        void getTf(const GjkSimplex& _i, const ClosestFeature& _cs);
    };
    
    bool eq(const ProjektConstraintNoPenetration::SubConstraint& a, const ProjektConstraintNoPenetration::SubConstraint& b);

    void activateImpulse() override;

    float calcApplyImpulse(float dt) override;
    
    void storeAccImpulse() override;
    
    float applyAccImpulse() override;

    float calcImpulse(SubConstraint & sc, float dt);

    void applyImpulse(SubConstraint & sc);

private:
    void reevalSc(SubConstraint& sc);

    std::vector<SubConstraint> getNewSubConstraints(const ClosestFeature& cs, const GjkSimplex& _i);
};

bool operator==(const ProjektConstraintNoPenetration::SubConstraint& a,
    const ProjektConstraintNoPenetration::SubConstraint& b);

class ClippingPlane
{
    std::array<VectorWithIndex, 2> incE, refE;
    const ProjektConvexPolygon  *incP, *refP;
    
public:
    ClippingPlane(const ProjektConvexPolygon& _incP, std::array<VectorWithIndex, 2> _incE,
        const ProjektConvexPolygon& _refP, std::array<VectorWithIndex, 2> _refE) :
        incP(&_incP), refP(&_refP), refE(_refE), incE(_incE) {}

    ClippingPlane(const ClosestFeature& cs, const GjkSimplex& _i,
        const ProjektConvexPolygon& pcpa, const ProjektConvexPolygon& pcpb);

    bool operator==(const ClippingPlane& cp);

    typedef struct { Vector2f point; float penetration; } PointAndPenetration;
    typedef std::optional < std::array<PointAndPenetration, 2> > PointsAndPenetrations;

    PointsAndPenetrations getPointsAndPenetrations();

};


class ProjektConstraintNoPWithFrict : public ProjektConstraintNoPenetration
{

};