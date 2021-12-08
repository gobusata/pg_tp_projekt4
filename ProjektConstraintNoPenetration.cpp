#include "ProjektConstraintNoPenetration.h"

float ProjektConstraintNoPenetration::friction_coeff = 0.2f;
float ProjektConstraintNoPenetration::beta = 1e-5f;
float ProjektConstraintNoPenetration::borderZoneWidth = 3.0f;

using SubConstraint = ProjektConstraintNoPenetration::SubConstraint;

ProjektConstraintNoPenetration::ProjektConstraintNoPenetration(ProjektConvexBody& _a, ProjektConvexBody& _b) :
    ProjektTwoBodyConstraint{ _a, _b }, subconstraints{ 2 }
{
    for (int i = 0; i < 3; i++)
    {
        kmat.diagonal()[i] = _a.kmat.diagonal()[i];
        kmat.diagonal()[i + 3] = _b.kmat.diagonal()[i];
    }
}

Vector2f ProjektConstraintNoPenetration::pointOfContact(const SubConstraint& sc) const
{
    //assert(sc.tfa != ProjektConstraintNoPenetration::edge || sc.tfb != ProjektConstraintNoPenetration::edge);
    if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return 0.5 * (a.pcp.getVertexPos(sc.cpa[0]) + b.pcp.getVertexPos(sc.cpb[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return a.pcp.getVertexPos(sc.cpa[0]);
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return b.pcp.getVertexPos(sc.cpb[0]);
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        Vector2f v1{ a.pcp.getVertexPos( sc.cpa[1]) - a.pcp.getVertexPos(sc.cpa[0]) },
            v2{ b.pcp.getVertexPos(sc.cpb[1]) - b.pcp.getVertexPos(sc.cpb[0]) };
        Vector2f va{ a.pcp.getVertexPos(sc.cpa[1]) }, vb;
        if (v1.dot(b.pcp.getVertexPos(sc.cpb[0])) < v1.dot(b.pcp.getVertexPos(sc.cpb[1])))
            vb = b.pcp.getVertexPos(sc.cpb[0]);
        else
            vb = b.pcp.getVertexPos(sc.cpb[1]);
        return (va * v1.norm() + vb * v2.norm()) / (v1.norm() + v2.norm());
    }
    return { 0, 0 };

}

float ProjektConstraintNoPenetration::getDistance(const SubConstraint& sc)
{
    if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return sc.normal.dot(b.pcp.getVertexPos(sc.cpb[0])) - sc.normal.dot(a.pcp.getVertexPos(sc.cpa[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return sc.normal.dot(b.pcp.getVertexPos(sc.cpb[0])) - sc.normal.dot(a.pcp.getVertexPos(sc.cpa[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return sc.normal.dot(b.pcp.getVertexPos(sc.cpb[0])) - sc.normal.dot(a.pcp.getVertexPos(sc.cpa[0]));
    }
    else//if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return sc.normal.dot( b.pcp.getVertexPos(sc.cpb[0]) - a.pcp.getVertexPos(sc.cpa[0]) );
    }
}

void ProjektConstraintNoPenetration::activateImpulse()
{
    if (collisionDetectionWide(a, b))
    {
        shapes_in_close_proximity = true;
        //intersection.simpstate = GjkSimplex::uninitialized;
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
                SubConstraint sc;
                sc.getTf(intersection, cs);
                sc.active = true;
                sc.constraint_error = borderZoneWidth - cs.distance;
                dbgmsg("Subconstraint:\nnormal = {:.2f}, point_of_contact = {:.2f}", sc.normal, pointOfContact(sc));

                //find redundant constraint
                for (SubConstraint& sci : subconstraints) 
                {
                    if (sc == sci || (pointOfContact(sc) - pointOfContact(sci)).norm() < borderZoneWidth)
                    {
                        sci.active = false;
                        break;
                    }
                }   
                //reevaluate constraints' erros and turn off invalid subconstraints
                for (SubConstraint& sci : subconstraints)
                {
                    if (sci.active)
                    {
                        float d = getDistance(sci);
                        if (d > borderZoneWidth+2)
                        {
                            sci.active = false;
                        }
                        else
                        {
                            sci.constraint_error = borderZoneWidth - d;
                        }
                    }
                }
                //find subconstraint with smallest constraint error and replace with new subconstraint
                SubConstraint* sc_max_dis = &subconstraints[0];
                for (SubConstraint& sci : subconstraints)
                {
                    if (!sci.active)
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
          /*  for (SubConstraint& sci : subconstraints)
            {
                if (sci.active)
                {
                    float d = get_distance(sci);
                    if (d > borderZoneWidth)
                    {
                        sci.active = false;
                    }
                    else
                    {
                        sci.constraint_error = borderZoneWidth - d;
                    }
                }
            }*/
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

float ProjektConstraintNoPenetration::calcImpulse(SubConstraint & sc, float dt)
{   
    float error = 0;
    if (active)
    {
        if (sc.active)
        {                
            sc.ln = -(sc.jmat.head<3>() * a.vel + sc.jmat.tail<3>() * b.vel)(0, 0) / (sc.jmat * kmat * sc.jmat.transpose())(0, 0);

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

bool operator==(const SubConstraint& a,
    const SubConstraint& b)
{
    if (a.tfa == b.tfa && a.tfb == b.tfb)
    {
        if (a.tfa == ProjektConstraintNoPenetration::edge)
        {
            if (a.tfb == ProjektConstraintNoPenetration::edge)
            {
                if (a.cpa[0] == b.cpa[0] && a.cpa[1] == b.cpa[1] && a.cpb[0] == b.cpb[0] && a.cpb[1] == b.cpb[1])
                    return true;
                else
                    return false;
            }
            else if (a.tfb == ProjektConstraintNoPenetration::vertex)
            {
                if (a.cpa[0] == b.cpa[0] && a.cpa[1] == b.cpa[1] && a.cpb[0] == b.cpb[0])
                    return true;
                else
                    return false;
            }
        }
        else if(a.tfa == ProjektConstraintNoPenetration::vertex)
        {
            if (a.tfb == ProjektConstraintNoPenetration::edge)
            {
                if (a.cpa[0] == b.cpa[0] && a.cpb[0] == b.cpb[0] && a.cpb[1] == b.cpb[1])
                    return true;
                else
                    return false;
            }
            else if (a.tfb == ProjektConstraintNoPenetration::vertex)
            {
                if (a.cpa[0] == b.cpa[0] &&a.cpb[0] == b.cpb[0])
                    return true;
                else
                    return false;
            }
        }
    }
    return false;
}

void ProjektConstraintNoPenetration::SubConstraint::getTf(const GjkSimplex & _i, const ClosestFeature & _cs)
{    
    if (_cs.feature.size() == 1)
    {
        tfa = TouchingFeature::vertex;
        tfb = TouchingFeature::vertex;
        cpa[0] = _i.simplex_vertices[_cs.feature[0].index].index;
        cpb[0] = _i.simplex_vertices[_cs.feature[0].index + 3].index;
        normal = _i.simplex_vertices[_cs.feature[0].index + 3] - _i.simplex_vertices[_cs.feature[0].index];
    }
    else if (_cs.feature.size() == 2)
    {
        cpa[0] = _i.simplex_vertices[_cs.feature[0].index].index;
        cpa[1] = _i.simplex_vertices[_cs.feature[1].index].index;
        cpb[0] = _i.simplex_vertices[_cs.feature[0].index + 3].index;
        cpb[1] = _i.simplex_vertices[_cs.feature[1].index + 3].index;
        if (cpa[0] == cpa[1]) 
            tfa = TouchingFeature::vertex;
        else 
            tfa = TouchingFeature::edge;
        if (cpb[0] == cpb[1]) 
            tfb = TouchingFeature::vertex;
        else 
            tfb = TouchingFeature::edge;
            
        if (tfa == TouchingFeature::edge && tfb == TouchingFeature::vertex)
        {
            normal = _i.simplex_vertices[_cs.feature[0].index] - _i.simplex_vertices[_cs.feature[1].index];
            normal = cross(normal, cross(_i.simplex_vertices[_cs.feature[0].index + 3] - _i.simplex_vertices[_cs.feature[0].index], normal));
        }
        else if (tfa == TouchingFeature::vertex && tfb == TouchingFeature::edge)
        {
            normal = _i.simplex_vertices[_cs.feature[0].index + 3] - _i.simplex_vertices[_cs.feature[1].index + 3];
            normal = cross(normal, cross(_i.simplex_vertices[_cs.feature[0].index + 3] - _i.simplex_vertices[_cs.feature[0].index], normal));
        }
        else if (tfa == TouchingFeature::edge && tfb == TouchingFeature::edge)
        {
            normal = _i.simplex_vertices[_cs.feature[0].index] - _i.simplex_vertices[_cs.feature[1].index];
            normal = cross(normal, cross(_i.simplex_vertices[_cs.feature[0].index + 3] - _i.simplex_vertices[_cs.feature[0].index], normal));
        }

    }

    normal.normalize();
}
