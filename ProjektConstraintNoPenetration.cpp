#include "ProjektConstraintNoPenetration.h"

ProjektConstraintNoPenetration::ProjektConstraintNoPenetration(UniversalConvexShape& _a, UniversalConvexShape& _b):
    a{ _a }, b{ _b }, j_mat(2, 6), minv_mat(6, 6), normal{}
{
    minv_mat.setZero();
    minv_mat(0, 0) = minv_mat(1, 1) = 1/a.mass;
    minv_mat(2, 2) = 1/a.inertia;
    minv_mat(3, 3) = minv_mat(4, 4) = 1/b.mass;
    minv_mat(5, 5) = 1/b.inertia;
}

void ProjektConstraintNoPenetration::activateImpulse()
{
    shapes_in_close_proximity = collisionDetectionWide(a, b); 
    if (shapes_in_close_proximity)
    {
        intersection = gjkSimplex(a, b);
        if (!intersection.collision)
        {
            if (intersection.aClosestFeature.size() == 2)
                if (intersection.aClosestFeature[0].index == intersection.aClosestFeature[1].index) intersection.aClosestFeature.pop_back();
            if (intersection.bClosestFeature.size() == 2)
                if (intersection.bClosestFeature[0].index == intersection.bClosestFeature[1].index) intersection.bClosestFeature.pop_back();
            if (intersection.aClosestFeature.size() == 2 && intersection.bClosestFeature.size() == 1)
            {
                normal = intersection.aClosestFeature[1] - intersection.aClosestFeature[0];
                normal = cross(cross(normal, intersection.bClosestFeature[0] - intersection.aClosestFeature[0]), normal);
                touching_shape = ucs_b;
                touching_vertex[0] = intersection.bClosestFeature[0].index;
            }
            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 2)
            {
                normal = intersection.bClosestFeature[1] - intersection.bClosestFeature[0];
                normal = -cross(cross(normal, intersection.aClosestFeature[0] - intersection.bClosestFeature[0]), normal);
                touching_shape = ucs_a;
                touching_vertex[0] = intersection.aClosestFeature[0].index;
            }
            else if (intersection.aClosestFeature.size() == 1 && intersection.bClosestFeature.size() == 1)
            {
                normal = intersection.bClosestFeature[0] - intersection.aClosestFeature[0];
                touching_vertex[0] = intersection.aClosestFeature[0].index;
                touching_vertex[1] = intersection.bClosestFeature[0].index;
            }
            else if (intersection.aClosestFeature.size() == 2 && intersection.bClosestFeature.size())
            {

                throw std::logic_error("My bad");
            }
            normal.normalize();
        }

        if (!intersection.collision)
        {
            if (intersection.distance < 4)
            {
                active = true;
                constraint_error = intersection.distance;
            }
            else
                active = false;
        }
        else
        {
            active = true;
            constraint_error = 4;
        }
    }
}

inline Vector2f ProjektConstraintNoPenetration::point_of_contact()
{       
    if (touching_shape == ucs_a)
        return a.getVertexPos(touching_vertex[0]);
    else if(touching_shape == ucs_b)
        return b.getVertexPos(touching_vertex[0]);
    else if (touching_shape == ucs_ab)
    {
        return 0.5 * (a.getVertexPos(touching_vertex[0]) + b.getVertexPos(touching_vertex[1]));
    }
}

float ProjektConstraintNoPenetration::calcImpulse(float dt)
{   
    Vector2f ra, rb;
    if(active)
    {
        Vector2f dir{ normal }, tan{ cross(1, dir) };

        ra = point_of_contact() - a.pos;
        rb = point_of_contact() - b.pos;

        j_mat(0, 0) = dir[0];   
        j_mat(0, 1) = dir[1];
        j_mat(0, 2) = dir[1] * ra[0] - dir[0] * ra[1];
        j_mat(0, 3) = -j_mat(0, 0);
        j_mat(0, 4) = -j_mat(0, 1);
        j_mat(0, 5) = dir[0] * rb[1] - dir[1] * rb[0];
        j_mat(1, 0) = tan[0];
        j_mat(1, 1) = tan[1];
        j_mat(1, 2) = tan[1] * ra[0] - tan[0] * ra[1];
        j_mat(1, 3) = -j_mat(1, 0);
        j_mat(1, 4) = -j_mat(1, 1);
        j_mat(1, 5) = tan[0] * rb[1] - tan[1] * rb[0];

        float cn, ct;
        cn = j_mat(0, 0) * a.vel[0] + j_mat(0, 1) * a.vel[1] + j_mat(0, 2) * a.omega
            + j_mat(0, 3) * b.vel[0] + j_mat(0, 4) * b.vel[1] + j_mat(0, 5) * b.omega
            + constraint_error / dt * beta;
        ct = j_mat(1, 0) * a.vel[0] + j_mat(1, 1) * a.vel[1] + j_mat(1, 2) * a.omega
            + j_mat(1, 3) * b.vel[0] + j_mat(1, 4) * b.vel[1] + j_mat(1, 5) * b.omega;

        Vector2f lambda = -(j_mat * minv_mat * j_mat.transpose()).inverse() * Vector2f { cn, ct };
        ln = lambda[0];
        lt = lambda[1];
        if (ln < 0)
            return ln * ln + lt * lt;
        else
            return 0.0f;
    }
    return 0.0f;
}

void ProjektConstraintNoPenetration::applyImpulse()
{
    if (active)
    {
        if (ln < 0)
        {
            a.vel[0] += (ln * j_mat(0, 0) + lt * j_mat(1, 0)) / a.mass;
            a.vel[1] += (ln * j_mat(0, 1) + lt * j_mat(1, 1)) / a.mass;
            a.omega += (ln * j_mat(0, 2) + lt * j_mat(1, 2)) / a.inertia;
            b.vel[0] += (ln * j_mat(0, 3) + lt * j_mat(1, 3)) / b.mass;
            b.vel[1] += (ln * j_mat(0, 4) + lt * j_mat(1, 4)) / b.mass;
            b.omega += (ln * j_mat(0, 5) + lt * j_mat(1, 5)) / b.inertia;
        }
    }
}

void ProjektConstraintNoPenetration::storeAccImpulse()
{
}

void ProjektConstraintNoPenetration::applyAccImpulse()
{
}
