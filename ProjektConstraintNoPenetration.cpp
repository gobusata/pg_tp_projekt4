#include "ProjektConstraintNoPenetration.h"

float ProjektConstraintNoPenetration::friction_coeff = 0.2f;
float ProjektConstraintNoPenetration::beta = 1.0e-4f;
float ProjektConstraintNoPenetration::borderZoneWidth = 3.0f;
float ProjektConstraintNoPenetration::pointWidth = 1e0;

using SubConstraint = ProjektConstraintNoPenetration::SubConstraint;
ClippingPlane::ClippingPlane(const ClosestFeature& cs, const GjkSimplex& _i, 
    const ProjektConvexPolygon& _pcpa, const ProjektConvexPolygon& _pcpb)
{
    enum{ edge, vertex } tfa, tfb;
    std::vector<SubConstraint> newSc;
    if (cs.feature.size() == 1)
    {
        tfa = vertex;
        tfb = vertex;
    }
    else if (cs.feature.size() == 2)
    {
        if (_i.simplex_vertices[cs.feature[0].index].index == _i.simplex_vertices[cs.feature[1].index].index)
            tfa = vertex;
        else
            tfa = edge;
        if (_i.simplex_vertices[cs.feature[0].index + 3].index == _i.simplex_vertices[cs.feature[1].index + 3].index)
            tfb = vertex;
        else
            tfb = edge;
    }

    if (tfa == vertex && tfb == edge)
    {
        refP = &_pcpb;
        incP = &_pcpa;
        arbi = false;
        refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
        refE[1] = _i.simplex_vertices[cs.feature[1].index + 3];
        incE[0] = _i.simplex_vertices[cs.feature[0].index];
        Vector2f e1{  incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size()) - 
            _i.simplex_vertices[cs.feature[0].index]},
            e2{ incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index].index - 1) % incP->vs.size()) - 
            _i.simplex_vertices[cs.feature[0].index]};
        normal = Vector2f( _i.simplex[cs.feature[0].index] - _i.simplex[cs.feature[1].index] );
        normal = cross(cross(normal, -cs.feature[0]), normal);
        normal.normalize();
        _n = -normal;
        float v1 = -normal.dot(e1), v2 = -normal.dot(e2);
        if (v1 < v2)
        {
            incE[0] = _i.simplex_vertices[cs.feature[0].index];
            int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size();
            incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
        }
        else
        {
            int ind = (_i.simplex_vertices[cs.feature[0].index].index - 1) % incP->vs.size();
            incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
            incE[1] = _i.simplex_vertices[cs.feature[0].index];
        }
        
        if ((refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
    }
    else if (tfa == edge && tfb == vertex)
    {
        refP = &_pcpa;
        incP = &_pcpb;
        arbi = true;
        refE[0] = _i.simplex_vertices[cs.feature[0].index];
        refE[1] = _i.simplex_vertices[cs.feature[1].index];
        Vector2f e1{ incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size()) - 
            _i.simplex_vertices[cs.feature[0].index + 3]},
            e2{ incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % incP->vs.size()) - 
            _i.simplex_vertices[cs.feature[0].index + 3]};
        normal = Vector2f{ _i.simplex[cs.feature[0].index] - _i.simplex[cs.feature[1].index] };
        normal = cross(cross(normal, -cs.feature[0]), normal);
        normal.normalize();
        _n = normal;
        float v1 = normal.dot(e1), v2 = normal.dot(e2);
        if (v1 < v2)
        {
            incE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
            int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size();
            incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
        }
        else
        {
            int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % incP->vs.size();
            incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
            incE[1] = _i.simplex_vertices[cs.feature[0].index + 3];
        }
        if ( (refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
    }
    else if (tfa == vertex && tfb == vertex)
    {
        normal = -cs.feature[0];
        normal.normalize();
        Vector2f e1{ _i.simplex_vertices[cs.feature[0].index] - _pcpa.getVertexPos((_i.simplex_vertices[cs.feature[0].index].index + 1) % _pcpa.vs.size()) },
                e2{ _i.simplex_vertices[cs.feature[0].index] - _pcpa.getVertexPos((_i.simplex_vertices[cs.feature[0].index].index - 1) % _pcpa.vs.size()) },
                e3{ _i.simplex_vertices[cs.feature[0].index + 3] - _pcpb.getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % _pcpb.vs.size()) },
                e4{ _i.simplex_vertices[cs.feature[0].index + 3] - _pcpb.getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % _pcpb.vs.size()) };
        
        float v1 = normal.dot(e1), v2 = normal.dot(e2), v3 = -normal.dot(e3), v4 = -normal.dot(e4);
        bool b1 = v1 < v2, b2 = v3 < v4, b3 = min(v1, v2) < min(v3, v4);
        if (b3)
        {
            refP = &_pcpa;
            incP = &_pcpb;
            arbi = true;
            _n = normal;
            if (b1)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % refP->vs.size();
                refE[0] = _i.simplex_vertices[cs.feature[0].index];
                refE[1] = VectorWithIndex(refP->getVertexPos(ind), ind);
                //ind = (_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % incP->vs.size();
                //incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
                //incE[1] = _i.simplex_vertices[cs.feature[0].index + 3];

            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index - 1) % refP->vs.size();
                refE[0] = VectorWithIndex(refP->getVertexPos(ind), ind);
                refE[1] = _i.simplex_vertices[cs.feature[0].index];
                //ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size();
                //incE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
                //incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
            }

            if (b2)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size();
                incE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
                incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % incP->vs.size();
                incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
                incE[1] = _i.simplex_vertices[cs.feature[0].index + 3];
            }
        }
        else
        {
            refP = &_pcpb;
            incP = &_pcpa;
            arbi = false;
            _n = -normal;
            if (b2)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % refP->vs.size();
                refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
                refE[1] = VectorWithIndex(refP->getVertexPos(ind), ind);
                //ind = (_i.simplex_vertices[cs.feature[0].index].index - 1) % incP->vs.size();
                //incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
                //incE[1] = _i.simplex_vertices[cs.feature[0].index];

            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % refP->vs.size();
                refE[0] = VectorWithIndex(refP->getVertexPos(ind), ind);
                refE[1] = _i.simplex_vertices[cs.feature[0].index + 3];
                //ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size();
                //incE[0] = _i.simplex_vertices[cs.feature[0].index];
                //incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);

            }

            if (b1)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size();
                incE[0] = _i.simplex_vertices[cs.feature[0].index];
                incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index - 1) % incP->vs.size();
                incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
                incE[1] = _i.simplex_vertices[cs.feature[0].index];
            }
        }
    }
    else if (tfa == edge && tfb == edge)
    {
        refP = &_pcpb;
        incP = &_pcpa;
        arbi = false;
        refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
        refE[1] = _i.simplex_vertices[cs.feature[1].index + 3];
        incE[0] = _i.simplex_vertices[cs.feature[0].index];
        incE[1] = _i.simplex_vertices[cs.feature[1].index];
        Vector2f tmp = incE[1] - incE[0];
        normal = cross(cross(tmp, -_i.simplex[0]), tmp);
        normal.normalize();
        _n = -normal;
        if ((refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
        if ((incE[0].index + 1) % incP->vs.size() != incE[1].index)
            std::swap(incE[0], incE[1]);
    }
}

bool ClippingPlane::operator==(const ClippingPlane& a)
{
    if (incE[0].index == a.incE[0].index && incE[1].index == a.incE[1].index &&
        refE[0].index == a.refE[0].index && refE[1].index == a.refE[1].index)
        return true;
    else if (incE[0].index == a.refE[0].index && incE[1].index == a.refE[1].index &&
        refE[0].index == a.incE[0].index && refE[1].index == a.incE[1].index)
        return true;
    else
        return false;
}


void ClippingPlane::reeval()
{
    incE[0] = incP->getVertexPos(incE[0].index);
    incE[1] = incP->getVertexPos(incE[1].index);
    refE[0] = refP->getVertexPos(refE[0].index);
    refE[1] = refP->getVertexPos(refE[1].index);
    _n = refE[1] - refE[0];
    _n = cross(cross(_n, refP->getTransformedVertex(refE[0].index)), _n);
    _n.normalize();
    normal = (arbi)?_n : -_n;
}

ClippingPlane::PointsAndPenetrations ClippingPlane::getPointsAndPenetrations()
{
    Vector2f dir = refE[1] - refE[0];
    dir.normalize();
    float tab[] = { dir.dot(incE[0]), dir.dot(incE[1]), dir.dot(refE[0]), dir.dot(refE[1]) };
    std::array<std::pair<uint8_t, float>, 4> stab{
        std::make_pair(0, tab[0]), std::make_pair(1, tab[1]),
        std::make_pair(2, tab[2]), std::make_pair(3, tab[3])
    };
    float w1 = normal.dot(incE[0]), w2 = normal.dot(incE[1]);
    std::sort(stab.begin(), stab.end(), [](auto a, auto b)->bool {return a.second < b.second; });
    PointsAndPenetrations paps;
    if (!(stab[0].first == 2 && stab[1].first == 3) && !(stab[2].first == 2 && stab[3].first == 3))
    {
        switch (stab[1].first)
        {
        case 0:
            paps.paps[0] = PointAndPenetration{ static_cast<Vector2f>(incE[0]), _n.dot(incE[0]) - _n.dot(refE[0]) };
            break;
        case 1:
            paps.paps[0] = PointAndPenetration{ static_cast<Vector2f>(incE[1]), _n.dot(incE[1]) - _n.dot(refE[1]) };
            break;
        case 2:
            paps.paps[0].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[2] - tab[0]) / (tab[1] - tab[0]),
                    (tab[1] - tab[2]) / (tab[1] - tab[0])));
            paps.paps[0].penetration = _n.dot(paps.paps[0].point) - _n.dot(refE[0]);
            break;
        case 3:
            paps.paps[0].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[3] - tab[0]) / (tab[1] - tab[0]),
                    (tab[1] - tab[3]) / (tab[1] - tab[0])));
            paps.paps[0].penetration = _n.dot(paps.paps[0].point) - _n.dot(refE[1]);
            break;
        }

        switch (stab[2].first)
        {
        case 0:
            paps.paps[1] = { incE[0], _n.dot(incE[0]) - _n.dot(refE[0])};
            break;
        case 1:
            paps.paps[1] = { incE[1], _n.dot(incE[1]) - _n.dot(refE[1])};
            break;
        case 2:
            paps.paps[1].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[2] - tab[0]) / (tab[1] - tab[0]),
                    (tab[1] - tab[2]) / (tab[1] - tab[0])));
            paps.paps[1].penetration = _n.dot(paps.paps[0].point) - _n.dot(refE[0]);
            break;
        case 3:
            paps.paps[1].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[3] - tab[0]) / (tab[1] - tab[0]),
                    (tab[1] - tab[3]) / (tab[1] - tab[0])));
            paps.paps[1].penetration = _n.dot(paps.paps[0].point) - _n.dot(refE[1]);
            break;
        }
        paps.on = true;        
    }
    else
    {
        paps.on = false;
    }
    return paps;
}

ProjektConstraintNoPenetration::ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b) :
    ProjektTwoBodyConstraint{ _a, _b }, subconstraints{SubConstraint(), SubConstraint()}
{
    for (int i = 0; i < 3; i++)
    {
        kmat.diagonal()[i] = _a.kmat.diagonal()[i];
        kmat.diagonal()[i + 3] = _b.kmat.diagonal()[i];
    }
}



void ProjektConstraintNoPenetration::activateImpulse()
{
    if (collisionDetectionWide(a, b))
    {
        shapes_in_close_proximity = true;
        intersection = gjkSimplex(a.pcp, b.pcp, intersection);
        if (!intersection.collision)
        {
            ClosestFeature cs;
            switch (intersection.simpstate)
            {
            case GjkSimplex::point:
                cs.feature.push_back(VectorWithIndex(intersection.simplex[0], 0));
                cs.distance = intersection.simplex[0].norm();
                break;
            case GjkSimplex::line:
                cs = pointToLine({ intersection.simplex[0], intersection.simplex[1] }, Vector2f(0, 0));
                break;
            case GjkSimplex::triangle:
                cs = pointToTriangle(intersection.simplex, Vector2f(0, 0));
                break;
            }

            if (cs.distance < borderZoneWidth)
            {
                active = true;
                ClippingPlane _clippingPlane(cs, intersection, a.pcp, b.pcp);
                ClippingPlane::PointsAndPenetrations paps = _clippingPlane.getPointsAndPenetrations();
                if (!(clippingPlane == _clippingPlane))
                {   
                    subconstraints[0].accln = 0;
                    subconstraints[1].acclt = 0;
                }
                clippingPlane = _clippingPlane;
                if (paps.on)
                {
                    auto scs = getSubConstraints(paps, clippingPlane);
                    reevalSubConstraints(subconstraints.begin(), subconstraints.end(), scs);
                }
                else
                {
                    active = false;
                    for (SubConstraint& sci : subconstraints)
                        sci.active = false;
                }
            }
            else
            {
                if (active)
                {
                    active = false;
                    for (SubConstraint& sc : subconstraints)
                        sc.active = false;
                }
            }
        }
        else
        {
            active = true;
            clippingPlane.reeval();
            ClippingPlane::PointsAndPenetrations paps = clippingPlane.getPointsAndPenetrations();
            if (paps.on)
            {
                std::array<SubConstraint, 2> scs = getSubConstraints(paps, clippingPlane);
                reevalSubConstraints(subconstraints.begin(), subconstraints.end(), scs);
            }
            else
            {
                active = false;
                for (SubConstraint& sc : subconstraints)
                    sc.active = false;

            }
        }
    }
    else
    {
        //if (active)
        //{
            active = false;
            for (SubConstraint& sc : subconstraints)
                sc.active = false;
        //}
        intersection.simpstate = GjkSimplex::uninitialized;
    }
}

float ProjektConstraintNoPenetration::calcImpulse(SubConstraint & sc, float dt)
{   
    float error = 0;
    sc.ln = -((sc.jmat.head<3>() * a.vel + sc.jmat.tail<3>() * b.vel)(0, 0) + beta * sc.constraint_error) / (sc.jmat * kmat * sc.jmat.transpose())(0, 0);

    if (sc.accln + sc.ln > 0)
        sc.ln = -sc.accln;

    sc.lt = 0;

    sc.accln += sc.ln;
    sc.acclt += sc.lt;

    if (sc.accln != 0)
        error = max(error, abs(sc.ln / sc.accln));
    if (sc.acclt != 0)
        error = max(error, abs(sc.lt / sc.acclt));
    return error;
}

void ProjektConstraintNoPenetration::applyImpulse(SubConstraint & sc)
{
    a.vel += sc.ln * a.kmat * sc.jmat.head<3>().transpose();
    b.vel += sc.ln * b.kmat * sc.jmat.tail<3>().transpose();
}

inline std::array<SubConstraint, 2> ProjektConstraintNoPenetration::getSubConstraints(const ClippingPlane::PointsAndPenetrations& paps, 
    const ClippingPlane& _cp)
{
    assert(paps.on);
    std::array<SubConstraint, 2> scs;
    scs[0].pointOfContact = paps.paps[0].point;
    scs[0].distance = paps.paps[0].penetration;
    if (scs[0].distance <= borderZoneWidth)
    {
        scs[0].constraint_error = getConstraintError(scs[0].distance);
        scs[0].active = true;
        scs[0].normal = _cp.normal;
    }
    else scs[0].active = false;
    scs[1].pointOfContact = paps.paps[1].point;
    scs[1].distance = paps.paps[1].penetration;
    if (scs[1].distance <= borderZoneWidth)
    {
        scs[1].constraint_error = getConstraintError(scs[1].distance);
        scs[1].active = true;
        scs[1].normal = _cp.normal;
    }
    else scs[1].active = false;
    if ((subconstraints[0].getPointOfContact() - subconstraints[1].getPointOfContact()).norm() < 1)
        subconstraints[1].active = false;
    return scs;
}

inline float ProjektConstraintNoPenetration::getConstraintError(float distance)
{
    return borderZoneWidth - distance;
}

float ProjektConstraintNoPenetration::calcApplyImpulse(float dt)
{
    if (active)
    {
        float err = 0;
        for (std::array<SubConstraint, 2>::iterator sci = subconstraints.begin(); sci != subconstraints.end(); sci++)
        {
            if (sci->active)
            {
                err = max(err, calcImpulse(*sci, dt));
                applyImpulse(*sci);
            }
        }
        return err;
    }
    return 0;
}

void ProjektConstraintNoPenetration::storeAccImpulse()
{
  
}

float ProjektConstraintNoPenetration::applyAccImpulse()
{
    if (active)
    {
        for (SubConstraint& sc : subconstraints)
        {
            if (sc.active)
            {
                Vector2f ra, rb;
                Vector2f dir{ sc.normal }, tan{ cross(1, dir) };
                ra = sc.getPointOfContact() - Vector2f{a.pos(0), a.pos(1)};
                rb = sc.getPointOfContact() - Vector2f{b.pos(0), b.pos(1)};

                sc.jmat(0, 0) = dir[0];
                sc.jmat(0, 1) = dir[1];
                sc.jmat(0, 2) = dir[1] * ra[0] - dir[0] * ra[1];
                sc.jmat(0, 3) = -sc.jmat(0, 0);
                sc.jmat(0, 4) = -sc.jmat(0, 1);
                sc.jmat(0, 5) = dir[0] * rb[1] - dir[1] * rb[0];

                if (warm_start)
                {
                    a.vel += a.kmat * sc.jmat.head<3>().transpose() * sc.accln;
                    b.vel += b.kmat * sc.jmat.tail<3>().transpose() * sc.accln;
                
                }
                else
                {
                    sc.accln = 0;
                    sc.acclt = 0;
                }
            }
        }
        return 1;
    }
}



