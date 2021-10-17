#include "ProjektConstraintNoPenetration.h"

ProjektConstraintNoPenetration::ProjektConstraintNoPenetration(UniversalConvexShape& _a, UniversalConvexShape& _b):
    a{ _a }, b{ _b }, minv_mat(6, 6), subconstraints{ 2 }
{
    minv_mat.setZero();
    minv_mat(0, 0) = minv_mat(1, 1) = 1/a.mass;
    minv_mat(2, 2) = 1/a.inertia;
    minv_mat(3, 3) = minv_mat(4, 4) = 1/b.mass;
    minv_mat(5, 5) = 1/b.inertia;
}

Vector2f ProjektConstraintNoPenetration::point_of_contact(const ProjektConstraintNoPenetration::SubConstraint& sc)
{
    assert(sc.tfa != ProjektConstraintNoPenetration::edge || sc.tfb != ProjektConstraintNoPenetration::edge);
    if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return 0.5 * (a.getVertexPos(sc.cpa[0]) + b.getVertexPos(sc.cpb[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return a.getVertexPos(sc.cpa[0]);
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return b.getVertexPos(sc.cpb[0]);
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        Vector2f v1{ a.getVertexPos( sc.cpa[1]) - a.getVertexPos(sc.cpa[0]) },
            v2{ b.getVertexPos(sc.cpb[1]) - b.getVertexPos(sc.cpb[0]) };
        Vector2f va{ a.getVertexPos(sc.cpa[1]) }, vb;
        if (v1.dot(b.getVertexPos(sc.cpb[0])) < v1.dot(b.getVertexPos(sc.cpb[1])))
            vb = b.getVertexPos(sc.cpb[0]);
        else
            vb = b.getVertexPos(sc.cpb[1]);
        return (va * v1.norm() + vb * v2.norm()) / (v1.norm() + v2.norm());
    }
    return { 0, 0 };

}
float ProjektConstraintNoPenetration::get_distance(const ProjektConstraintNoPenetration::SubConstraint& sc)
{
    if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return sc.normal.dot(a.getVertexPos(sc.cpb[0])) - sc.normal.dot(b.getVertexPos(sc.cpa[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::vertex)
    {
        return sc.normal.dot(a.getVertexPos(sc.cpb[0])) - sc.normal.dot(b.getVertexPos(sc.cpa[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::vertex && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return sc.normal.dot(a.getVertexPos(sc.cpb[0])) - sc.normal.dot(b.getVertexPos(sc.cpa[0]));
    }
    else if (sc.tfa == ProjektConstraintNoPenetration::edge && sc.tfb == ProjektConstraintNoPenetration::edge)
    {
        return sc.normal.dot(a.getVertexPos(sc.cpb[0])) - sc.normal.dot(b.getVertexPos(sc.cpa[0]));
    }
}
//
//void ProjektConstraintNoPenetration::activateImpulse()
//{
//    shapes_in_close_proximity = collisionDetectionWide(a, b); 
//    if (shapes_in_close_proximity)
//    {
//        intersection = gjkSimplex(a, b);
//        if (!intersection.collision)
//        {
//            if (intersection.aClosestFeature.size() == 2)
//                if (intersection.aClosestFeature[0].index == intersection.aClosestFeature[1].index) intersection.aClosestFeature.pop_back();
//            if (intersection.bClosestFeature.size() == 2)
//                if (intersection.bClosestFeature[0].index == intersection.bClosestFeature[1].index) intersection.bClosestFeature.pop_back();
//            if (intersection.aClosestFeature.size() == 2 && intersection.bClosestFeature.size() == 1)
//            {
//                normal = intersection.aClosestFeature[1] - intersection.aClosestFeature[0];
//                normal = cross(cross(normal, intersection.bClosestFeature[0] - intersection.aClosestFeature[0]), normal);
//                touching_shape = ucs_b;
//                touching_vertex[0] = intersection.bClosestFeature[0].index;
//            }
//            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 2)
//            {
//                normal = intersection.bClosestFeature[1] - intersection.bClosestFeature[0];
//                normal = -cross(cross(normal, intersection.aClosestFeature[0] - intersection.bClosestFeature[0]), normal);
//                touching_shape = ucs_a;
//                touching_vertex[0] = intersection.aClosestFeature[0].index;
//            }
//            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 1)
//            {
//                normal = intersection.bClosestFeature[0] - intersection.aClosestFeature[0];
//                touching_vertex[0] = intersection.aClosestFeature[0].index;
//                touching_vertex[1] = intersection.bClosestFeature[0].index;
//            }
//            else if (intersection.aClosestFeature.size() == 2 && intersection.bClosestFeature.size() == 2)
//            {
//
//                throw std::logic_error("My bad");
//            }
//            normal.normalize();
//        }
//
//        if (!intersection.collision)
//        {
//            if (intersection.distance < 4)
//            {
//                active = true;
//                constraint_error = intersection.distance;
//            }
//            else
//                active = false;
//        }
//        else
//        {
//            active = true;
//            constraint_error = 4;
//        }
//    }
//}
//
//inline Vector2f ProjektConstraintNoPenetration::point_of_contact()
//{       
//    if (touching_shape == ucs_a)
//        return a.getVertexPos(touching_vertex[0]);
//    else if(touching_shape == ucs_b)
//        return b.getVertexPos(touching_vertex[0]);
//    else if (touching_shape == ucs_ab)
//    {
//        return 0.5 * (a.getVertexPos(touching_vertex[0]) + b.getVertexPos(touching_vertex[1]));
//    }
//}

void ProjektConstraintNoPenetration::activateImpulse()
{
    if (collisionDetectionWide(a, b))
    {
        shapes_in_close_proximity = true;
        intersection = gjkSimplex(a, b);
        if (!intersection.collision)
        {
            if (intersection.aClosestFeature.size() == 2)
                if (intersection.aClosestFeature[0].index == intersection.aClosestFeature[1].index)
                    intersection.aClosestFeature.pop_back();
            if (intersection.bClosestFeature.size() == 2)
                if (intersection.bClosestFeature[0].index == intersection.bClosestFeature[1].index)
                    intersection.bClosestFeature.pop_back();
            //assert(intersection.aClosestFeature.size() != 2 || intersection.bClosestFeature.size() != 2);
            ProjektConstraintNoPenetration::SubConstraint sc;
            if (intersection.aClosestFeature.size() == 2 && intersection.bClosestFeature.size() == 1)
            {
                sc.normal = intersection.aClosestFeature[1] - intersection.aClosestFeature[0];
                sc.normal = cross(cross(sc.normal, intersection.bClosestFeature[0] - intersection.aClosestFeature[0]), sc.normal);
                sc.tfa = ProjektConstraintNoPenetration::edge;
                sc.tfb = ProjektConstraintNoPenetration::vertex;
                sc.cpa[0] = intersection.aClosestFeature[0].index;
                sc.cpa[1] = intersection.aClosestFeature[1].index;
                sc.cpb[0] = intersection.bClosestFeature[0].index;
            }
            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 2)
            {
                sc.normal = intersection.bClosestFeature[1] - intersection.bClosestFeature[0];
                sc.normal = -cross(cross(sc.normal, intersection.aClosestFeature[0] - intersection.bClosestFeature[0]), sc.normal);
                sc.tfa = ProjektConstraintNoPenetration::vertex;
                sc.tfb = ProjektConstraintNoPenetration::edge;
                sc.cpa[0] = intersection.aClosestFeature[0].index;
                sc.cpb[0] = intersection.bClosestFeature[0].index;
                sc.cpb[1] = intersection.bClosestFeature[1].index;
            }
            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 1)
            {
                sc.normal = intersection.bClosestFeature[0] - intersection.aClosestFeature[0];
                sc.tfa = ProjektConstraintNoPenetration::vertex;
                sc.tfb = ProjektConstraintNoPenetration::vertex;
                sc.cpa[0] = intersection.aClosestFeature[0].index;
                sc.cpb[0] = intersection.bClosestFeature[0].index;
            }

            //sc.constraint_error = 20 - intersection.distance;
            //sc.valid = true;
            //subconstraints[0] = sc;
            if (intersection.distance < 20)
                active = true;
            sc.valid = true;
            for (ProjektConstraintNoPenetration::SubConstraint& sc : subconstraints)
            {
                if (sc.valid)
                {
                    float d = get_distance(sc);
                    if (d > 30)
                    {
                        sc.valid = false;
                    }
                    else
                    {
                        sc.constraint_error = 20 - d;
                    }
                }
            }

            ProjektConstraintNoPenetration::SubConstraint* sc_max_dis = &subconstraints[0];
            for (ProjektConstraintNoPenetration::SubConstraint& sci : subconstraints) 
            {
                if (!sc.valid)
                {
                    sc_max_dis = &sci;
                    break;
                }
                else if(sc_max_dis->constraint_error < sci.constraint_error)
                {
                    if(!(sc == sci))
                        sc_max_dis = &sci;
                }
            }
            *sc_max_dis = sc;
            //if (subconstraints[0].valid && subconstraints[1].valid)
            if(true)
            {
                spdlog::get("basic_logger")->info("cp1 {}, cp2 {}", 
                    point_of_contact(subconstraints[0]), point_of_contact(subconstraints[1]));
            }
        }
        else
        {
            active = true;
        }
    }
    else
    {
        active = false;
    }
}

float ProjektConstraintNoPenetration::calcImpulse(float dt)
{   
    float lambda_sum = 0;
    if (active)
    {
        ProjektConstraintNoPenetration::SubConstraint sc;
        for (ProjektConstraintNoPenetration::SubConstraint& sc : subconstraints)
        {
            Vector2f ra, rb;
            Vector2f dir{ sc.normal }, tan{ cross(1, dir) };
            ra = point_of_contact(sc) - a.pos;
            rb = point_of_contact(sc) - b.pos;

            sc.j_mat(0, 0) = dir[0];
            sc.j_mat(0, 1) = dir[1];
            sc.j_mat(0, 2) = dir[1] * ra[0] - dir[0] * ra[1];
            sc.j_mat(0, 3) = -sc.j_mat(0, 0);
            sc.j_mat(0, 4) = -sc.j_mat(0, 1);
            sc.j_mat(0, 5) = dir[0] * rb[1] - dir[1] * rb[0];
            sc.j_mat(1, 0) = tan[0];
            sc.j_mat(1, 1) = tan[1];
            sc.j_mat(1, 2) = tan[1] * ra[0] - tan[0] * ra[1];
            sc.j_mat(1, 3) = -sc.j_mat(1, 0);
            sc.j_mat(1, 4) = -sc.j_mat(1, 1);
            sc.j_mat(1, 5) = tan[0] * rb[1] - tan[1] * rb[0];

            float cn, ct;
            cn = sc.j_mat(0, 0) * a.vel[0] + sc.j_mat(0, 1) * a.vel[1] + sc.j_mat(0, 2) * a.omega
                + sc.j_mat(0, 3) * b.vel[0] + sc.j_mat(0, 4) * b.vel[1] + sc.j_mat(0, 5) * b.omega
                + sc.constraint_error / dt * beta;
            ct = sc.j_mat(1, 0) * a.vel[0] + sc.j_mat(1, 1) * a.vel[1] + sc.j_mat(1, 2) * a.omega
                + sc.j_mat(1, 3) * b.vel[0] + sc.j_mat(1, 4) * b.vel[1] + sc.j_mat(1, 5) * b.omega;

            Vector2f lambda = -(sc.j_mat * minv_mat * sc.j_mat.transpose()).inverse() * Vector2f { cn, ct };
            sc.ln = lambda[0];
            sc.lt = lambda[1];
            //impulse clamping
            if (sc.acc_ln + sc.ln > 0)
                sc.ln = -sc.acc_ln;

            sc.acc_ln += sc.ln;
            sc.acc_lt += sc.lt;

            lambda_sum += sc.ln * sc.ln + sc.lt * sc.lt;
        }
    }
   
    return lambda_sum;
}

void ProjektConstraintNoPenetration::applyImpulse()
{
    if (active)
    {
        for (ProjektConstraintNoPenetration::SubConstraint& sc : subconstraints)
        {
            if (sc.ln < 0)
            {
                a.vel[0] += (sc.ln * sc.j_mat(0, 0) + sc.lt * sc.j_mat(1, 0)) / a.mass;
                a.vel[1] += (sc.ln * sc.j_mat(0, 1) + sc.lt * sc.j_mat(1, 1)) / a.mass;
                a.omega += (sc.ln * sc.j_mat(0, 2) + sc.lt * sc.j_mat(1, 2)) / a.inertia;
                b.vel[0] += (sc.ln * sc.j_mat(0, 3) + sc.lt * sc.j_mat(1, 3)) / b.mass;
                b.vel[1] += (sc.ln * sc.j_mat(0, 4) + sc.lt * sc.j_mat(1, 4)) / b.mass;
                b.omega += (sc.ln * sc.j_mat(0, 5) + sc.lt * sc.j_mat(1, 5)) / b.inertia;
            }
        }
    }
}

void ProjektConstraintNoPenetration::storeAccImpulse()
{

}

void ProjektConstraintNoPenetration::applyAccImpulse()
{
    if (active)
    {
        for (ProjektConstraintNoPenetration::SubConstraint& sc : subconstraints)
        {
            sc.acc_ln = 0;
            sc.acc_lt = 0;
        }
    }
}

bool ProjektConstraintNoPenetration::eq(const ProjektConstraintNoPenetration::SubConstraint& a, const ProjektConstraintNoPenetration& b)
{
    
    return false;
}

bool operator==(const ProjektConstraintNoPenetration::SubConstraint& a,
    const ProjektConstraintNoPenetration::SubConstraint& b)
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
    else
        return false;
}
