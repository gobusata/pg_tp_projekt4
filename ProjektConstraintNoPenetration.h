#pragma once
#include <stdexcept>
#include <optional>
#include <functional>
#include "ProjektConstraint.h"
#include "UniversalConvexShape.h"

class ClippingPlane
{
public:
    std::array<VectorWithIndex, 2> incE, refE;
    const ProjektConvexPolygon *incP, *refP;
    bool arbi;
    Vector2f _n;

public:
    ClippingPlane() : incE{ VectorWithIndex(), VectorWithIndex() }, refE{ VectorWithIndex(), VectorWithIndex() },
        incP(nullptr), refP(nullptr), normal(), _n() {};

    ClippingPlane(const ClosestFeature& cs, const GjkSimplex& _i,
        const ProjektConvexPolygon& pcpa, const ProjektConvexPolygon& pcpb);

    bool operator==(const ClippingPlane& cp);

    struct PointAndPenetration { Vector2f point; float penetration; };
    struct PointsAndPenetrations {
        std::array<PointAndPenetration, 2> paps;
        bool on;
    };

    Vector2f normal;

    void reeval();

    PointsAndPenetrations getPointsAndPenetrations();
};

class ProjektConstraintNoPenetration :
    public ProjektTwoBodyConstraint
{
public:
    class SubConstraint
    {
    public:
        Vector2f normal{ 0, 0 }, pointOfContact{0, 0};
        float ln{ 0 }, lt{ 0 }, accln{ 0 }, acclt{ 0 }, constraint_error{ 0 }, distance{ 0 };
        bool active{ false };
        Eigen::Matrix<float, 1, 6> jmat;

        SubConstraint(Vector2f _pointOfContact = Vector2f(), float _distance = 0.0f, float _constraint_error = 0.0f) :
            pointOfContact(_pointOfContact), distance(_distance), constraint_error{ _constraint_error } {}

        Vector2f getPointOfContact() const { return pointOfContact; };

        float getDistance() { return distance; };
    };
    typedef std::array<SubConstraint, 2> SubConstraintsT;
    SubConstraintsT subconstraints;
    GjkSimplex intersection;
    bool active;
    ClippingPlane clippingPlane;
private:
    static float friction_coeff, beta, borderZoneWidth, pointWidth;
    bool shapes_in_close_proximity;    
public:

    ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b);
    
    void activateImpulse() override;

    float calcApplyImpulse(float dt) override;
    
    void storeAccImpulse() override;
    
    float applyAccImpulse() override;

    float calcImpulse(SubConstraint & sc, float dt);

    void applyImpulse(SubConstraint & sc);

private:
    inline std::array<SubConstraint, 2> getSubConstraints(const ClippingPlane::PointsAndPenetrations& paps, 
        const ClippingPlane& _cp);

    inline float getConstraintError(float distance);

    template<int _n>
    inline void reevalSubConstraints(SubConstraintsT::iterator beg, SubConstraintsT::iterator end, std::array<SubConstraint, _n> scs)
    {
        for (int i = 0; i<_n && beg != end; i++, beg++)
        {
            beg->pointOfContact = scs[i].getPointOfContact();
            beg->normal = scs[i].normal;
            beg->distance = scs[i].getDistance();
            beg->active = scs[i].active;
            beg->constraint_error = scs[i].constraint_error;
        }
    }
};

class ProjektConstraintNoPWithFrict : public ProjektConstraintNoPenetration
{

};