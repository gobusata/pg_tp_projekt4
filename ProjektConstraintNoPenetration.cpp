#include "ProjektConstraintNoPenetration.h"

float ProjektConstraintNoPenetration::friction_coeff = 0.2f;
float ProjektConstraintNoPenetration::beta = 0e-4f;
float ProjektConstraintNoPenetration::borderZoneWidth = 6.0f;
float ProjektConstraintNoPenetration::pointWidth = 1e0;

using SubConstraint = ProjektConstraintNoPenetration::SubConstraint;

ProjektConstraintNoPenetration::ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b) :
    ProjektTwoBodyConstraint{ _a, _b }, subconstraints( std::size_t(2) )
{
    for (int i = 0; i < 3; i++)
    {
        kmat.diagonal()[i] = _a.kmat.diagonal()[i];
        kmat.diagonal()[i + 3] = _b.kmat.diagonal()[i];
    }
}

float ProjektConstraintNoPenetration::getDistance(const SubConstraint& sc)
{
    
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
                    subconstraints.clear();
                    subconstraints.push_back(SubConstraint(paps.paps[0].point, paps.paps[0].penetration, borderZoneWidth - paps.paps[1].penetration))
                }
                else
                {
                    subconstraints[0].pointOfContact = paps.paps[0].point;
                    subconstraints[0].distance = paps.paps[0].penetration;
                    subconstraints[1].pointOfContact = paps.paps[1].point;
                    subconstraints[1].distance = paps.paps[1].penetration;
                    
                }
                //reevaluate constraints' erros and turn off invalid subconstraints
                for (SubConstraint& sci : subconstraints)
                {
                    reevalSc(sci);
                    if (sci.inBounds)
                    {
                        float d = getDistance(sci);
                        if (d > borderZoneWidth)
                        {
                            sci.active = false;
                        }   
                        else
                        {
                            sci.active = true;
                            sci.constraint_error = max(borderZoneWidth - d, 0);
                        }
                    }
                }
                //find redundant constraint
                for (SubConstraint& sc : newSc)
                {
                    for (SubConstraint& sci : subconstraints) 
                    {
                        if (sci.valid)
                        {
                            if (sc == sci || (pointOfContact(sc) - pointOfContact(sci)).norm() < borderZoneWidth)
                            {
                                sc.inBounds = false;
                                sc.valid = false;
                                sc.active = false;
                                break;
                            }
                        }
                    }   
                }
                //find subconstraint with smallest constraint error and replace with new subconstraint
                for (; !newSc.empty(); newSc.pop_back())
                {
                    SubConstraint& sc = newSc.back();
                    if (sc.active)
                    {
                        SubConstraint* sc_max_dis = &subconstraints[0];
                        for (SubConstraint& sci : subconstraints)
                        {
                            if (!sci.valid)
                            {
                                sc_max_dis = &sci;
                                break;
                            }
                            else if (sc_max_dis->constraint_error > sci.constraint_error)
                            {
                                if (!(sc == sci))
                                    sc_max_dis = &sci;
                            }
                        }
                        *sc_max_dis = sc;
                    }
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
            for (SubConstraint& sci : subconstraints)
            {
                reevalSc(sci);
                if (sci.inBounds)
                {
                    float d = getDistance(sci);
                    if (d > borderZoneWidth)
                    {
                        sci.active = false;
                    }
                    else
                    {
                        sci.active = true;
                        sci.constraint_error = borderZoneWidth - d;
                    }
                    //dbgmsg("Subconstraint:\nnormal = {:.2f}, point_of_contact = {:.2f}", sc.normal, pointOfContact(sc));
                }
            }
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

        intersection.simpstate = GjkSimplex::uninitialized;
    }
}

float ProjektConstraintNoPenetration::calcImpulse(SubConstraint & sc, float dt)
{   
    float error = 0;
    if (active)
    {
        if (sc.active)
        {                
            sc.ln = -((sc.jmat.head<3>() * a.vel + sc.jmat.tail<3>() * b.vel)(0, 0) + beta * sc.constraint_error) / (sc.jmat * kmat * sc.jmat.transpose())(0, 0);

            if (sc.acc_ln + sc.ln > 0)
                sc.ln = -sc.acc_ln;

            sc.lt = 0;

            sc.acc_ln += sc.ln;
            sc.acc_lt += sc.lt;

            if (sc.acc_ln != 0)
                error = max(error, abs(sc.ln / sc.acc_ln));
            if (sc.acc_lt != 0)
                error = max(error, abs(sc.lt / sc.acc_lt));
        }
    }
    return error;
}

void ProjektConstraintNoPenetration::applyImpulse(SubConstraint & sc)
{
    if (active)
    {
        if (sc.active)
        {
            a.vel += sc.ln * a.kmat * sc.jmat.head<3>().transpose();
            b.vel += sc.ln * b.kmat * sc.jmat.tail<3>().transpose();
        }
    }
}

float ProjektConstraintNoPenetration::calcApplyImpulse(float dt)
{
    float err = 0;
    for (std::vector<SubConstraint>::iterator sci = subconstraints.begin(); sci != subconstraints.end(); sci++)
    {
        err = max(err, calcImpulse(*sci, dt));
        applyImpulse(*sci);
    }
    return err;
}

void ProjektConstraintNoPenetration::storeAccImpulse()
{
  
}

float ProjektConstraintNoPenetration::applyAccImpulse()
{
    for (SubConstraint& sc : subconstraints)
    {
        if (sc.active)
        {
            Vector2f ra, rb;
            Vector2f dir{ sc.normal }, tan{ cross(1, dir) };
            ra = pointOfContact(sc) - Vector2f{ a.pos(0), a.pos(1) };
            rb = pointOfContact(sc) - Vector2f{ b.pos(0), b.pos(1) };

            sc.jmat(0, 0) = dir[0];
            sc.jmat(0, 1) = dir[1];
            sc.jmat(0, 2) = dir[1] * ra[0] - dir[0] * ra[1];
            sc.jmat(0, 3) = -sc.jmat(0, 0);
            sc.jmat(0, 4) = -sc.jmat(0, 1);
            sc.jmat(0, 5) = dir[0] * rb[1] - dir[1] * rb[0];

            if (warm_start)
            {
                a.vel += a.kmat * sc.jmat.head<3>().transpose() * sc.acc_ln;
                b.vel += b.kmat * sc.jmat.tail<3>().transpose() * sc.acc_ln;
                
            }
            else
            {
                sc.acc_ln = 0;
                sc.acc_lt = 0;
            }
        }
    }
    return 1;
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

ClippingPlane::ClippingPlane(const ClosestFeature& cs, const GjkSimplex& _i, 
    const ProjektConvexPolygon& pcpa, const ProjektConvexPolygon& pcpb)
{
    enum { edge, vertex } tfa, tfb;
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
        refP = &pcpb;
        incP = &pcpa;
        refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
        refE[1] = _i.simplex_vertices[cs.feature[1].index + 3];
        incE[0] = _i.simplex_vertices[cs.feature[0].index];
        Vector2f e1{ _i.simplex_vertices[cs.feature[0].index] -
            incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size()) },
            e2{ _i.simplex_vertices[cs.feature[1].index] -
            incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index].index - 1) % incP->vs.size()) };
        Vector2f normal{ _i.simplex[cs.feature[0].index] - _i.simplex[cs.feature[1].index] };
        normal = cross(cross(normal, -cs.feature[0]), normal);
        float v1 = normal.dot(e1), v2 = normal.dot(e2);
        if (v1 < v2)
        {
            incE[0] = _i.simplex_vertices[cs.feature[0].index];
            int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size();
            incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
        }
        else
        {
            int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % incP->vs.size();
            incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
            incE[1] = _i.simplex_vertices[cs.feature[0].index];
        }
        
        if ((refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
    }
    else if (tfa == edge && tfb == vertex)
    {
        refP = &pcpa;
        incP = &pcpb;
        refE[0] = _i.simplex_vertices[cs.feature[0].index];
        refE[1] = _i.simplex_vertices[cs.feature[1].index];
        Vector2f e1{ _i.simplex_vertices[cs.feature[0].index] -
          incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size()) },
            e2{ _i.simplex_vertices[cs.feature[1].index] -
            incP->getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % incP->vs.size()) };
        Vector2f normal{ _i.simplex[cs.feature[0].index] - _i.simplex[cs.feature[1].index] };
        normal = cross(cross(normal, -cs.feature[0]), normal);
        float v1 = normal.dot(e1), v2 = normal.dot(e2);
        if (v1 < v2)
        {
            incE[0] = _i.simplex_vertices[cs.feature[0].index];
            int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size();
            incE[1] = VectorWithIndex(incP->getVertexPos(ind), ind);
        }
        else
        {
            int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % incP->vs.size();
            incE[0] = VectorWithIndex(incP->getVertexPos(ind), ind);
            incE[1] = _i.simplex_vertices[cs.feature[0].index];
        }
        if ( (refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
    }
    else if (tfa == vertex && tfb == vertex)
    {
        Vector2f normal = -cs.feature[0];
        Vector2f e1{ _i.simplex_vertices[cs.feature[0].index] - pcpa.getVertexPos((_i.simplex_vertices[cs.feature[0].index].index + 1) % pcpa.vs.size()) },
                e2{ _i.simplex_vertices[cs.feature[0].index] - pcpa.getVertexPos((_i.simplex_vertices[cs.feature[0].index].index - 1) % pcpa.vs.size()) },
                e3{ _i.simplex_vertices[cs.feature[0].index + 3] - pcpb.getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % pcpb.vs.size()) },
                e4{ _i.simplex_vertices[cs.feature[0].index + 3] - pcpb.getVertexPos((_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % pcpb.vs.size()) };
        
        float v1 = normal.dot(e1), v2 = normal.dot(e2), v3 = normal.dot(e3), v4 = normal.dot(e4);
        bool b1 = v1 < v2, b2 = v3 < v4, b3 = min(v1, v2) < min(v3, v4);
        if (b3)
        {
            refP = &pcpa;
            incP = &pcpb;
            if (b1)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index + 1) % refP->vs.size();
                refE[0] = _i.simplex_vertices[cs.feature[0].index];
                refE[1] = VectorWithIndex(refP->getVertexPos(ind), ind);
            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index].index - 1) % refP->vs.size();
                refE[0] = VectorWithIndex(refP->getVertexPos(ind), ind);
                refE[1] = _i.simplex_vertices[cs.feature[0].index];
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
            refP = &pcpb;
            incP = &pcpa;
            if (b2)
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index + 1) % refP->vs.size();
                refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
                refE[1] = VectorWithIndex(refP->getVertexPos(ind), ind);
            }
            else
            {
                int ind = (_i.simplex_vertices[cs.feature[0].index + 3].index - 1) % refP->vs.size();
                refE[0] = VectorWithIndex(refP->getVertexPos(ind), ind);
                refE[1] = _i.simplex_vertices[cs.feature[0].index + 3];
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
        refE[0] = _i.simplex_vertices[cs.feature[0].index + 3];
        refE[1] = _i.simplex_vertices[cs.feature[1].index + 3];
        incE[0] = _i.simplex_vertices[cs.feature[0].index];
        incE[1] = _i.simplex_vertices[cs.feature[1].index];
        if ((refE[0].index + 1) % refP->vs.size() != refE[1].index)
            std::swap(refE[0], refE[1]);
        if ((incE[0].index + 1) % incP->vs.size() != incE[1].index)
            std::swap(incE[0], incE[1]);
    }
}

ClippingPlane::PointsAndPenetrations ClippingPlane::getPointsAndPenetrations()
{
    Vector2f dir = refE[1] - refE[0];
    Vector2f normal = dir;
    normal = cross(cross(normal, refE[0]), normal);
    normal.normalize();
    dir.normalize();
    std::array<std::pair<uint8_t, float>, 4> tab{
        std::make_pair(0, dir.dot(incE[0])), std::make_pair(1, dir.dot(incE[1])),
        std::make_pair(2, dir.dot(refE[0])), std::make_pair(3, dir.dot(refE[1]))
    };
    float w1 = normal.dot(incE[0]), w2 = normal.dot(incE[1]);
    std::sort(tab.begin(), tab.end(), [](auto a, auto b)->bool {return a.second() < b.second(); });
    PointsAndPenetrations paps;
    if (!(tab[0].first == 2 && tab[1].first == 3) && !(tab[2].first == 2 && tab[3].first == 3))
    {
        switch (tab[1].first)
        {
        case 0:
            paps.paps[0] = PointAndPenetration{ static_cast<Vector2f>(incE[0]), normal.dot(incE[0]) };
            break;
        case 1:
            paps.paps[0] = PointAndPenetration{ static_cast<Vector2f>(incE[1]), normal.dot(incE[1]) };
            break;
        case 2:
            paps.paps[0].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[2].second - tab[0].second) / (tab[1].second - tab[0].second),
                    (tab[1].second - tab[2].second) / (tab[1].second - tab[0].second)));
            break;
        case 3:
            paps.paps[0].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[3].second - tab[0].second) / (tab[1].second - tab[0].second),
                    (tab[1].second - tab[3].second) / (tab[1].second - tab[0].second)));
            break;
        }

        switch (tab[2].first)
        {
        case 0:
            paps.paps[1] = { incE[0], normal.dot(incE[0]) };
            break;
        case 1:
            paps.paps[1] = { incE[1], normal.dot(incE[1]) };
            break;
        case 2:
            paps.paps[1].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[2].second - tab[0].second) / (tab[1].second - tab[0].second),
                    (tab[1].second - tab[2].second) / (tab[1].second - tab[0].second)));
            break;
        case 3:
            paps.paps[1].point = cartesianCoordinates({ incE[0], incE[1] },
                Vector2f((tab[3].second - tab[0].second) / (tab[1].second - tab[0].second),
                    (tab[1].second - tab[3].second) / (tab[1].second - tab[0].second)));
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


