#pragma once

#include <crl-basic/utils/mathDefs.h>

#include <cfloat>

namespace crl::loco {

/**
 * returns the angle between between this vector and v. Given a direction n, the angle returned is between -PI and PI.
 */
inline double angleWith(const V3D& v1, const V3D& v2, const V3D& n) {
    // U.V = |U|*|V|*cos(angle)
    double cosAVal = v1.dot(v2) / (v1.norm() * v2.norm());
    V3D crossProd = v1.cross(v2);
    // |U x V| = |U|*|V|*sin(angle)
    double sinAVal = crossProd.norm() / (v1.norm() * v2.norm());
    if (crossProd.dot(n) < 0)
        sinAVal *= -1;
    return atan2(sinAVal, cosAVal);
}

/**
 * Given a set of points, this method 1: projects them to a plane defined
 * by normal n, and 2: returns a set of indices corresponding to the
 * points that define the planar convex hull of the set of points.
 */
static DynamicArray<int> planarConvexHullFromSetOfPoints(const DynamicArray<P3D>& pointSet, const V3D& n) {
    // first we gotta get two vectors orthogonal to n - we'll be encoding the
    // coordinates of the initial set of points in this basis
    V3D t1, t2;
    getOrthogonalVectors(n, t1, t2);

    DynamicArray<int> ordering;
    DynamicArray<P3D> points = pointSet;

    ordering.clear();

    //find the point that is "left-most", and start from there...
    double minVal = DBL_MAX;
    double min2ndVal = DBL_MAX;
    int index = 0;
    for (uint i = 0; i < points.size(); i++) {
        ordering.push_back(i);
        double val = V3D(points[i]).dot(t1);
        double val2 = V3D(points[i]).dot(t2);
        if (minVal > val || (fabs(minVal - val) < 0.000001 && val2 < min2ndVal)) {
            minVal = val;
            index = i;
            min2ndVal = val2;
        }
    }

    if (points.size() == 0)
        return ordering;

    //now, make sure we start from this point...
    P3D p = points[index];
    points[index] = points[0];
    points[0] = p;
    int tmpI = ordering[index];
    ordering[index] = ordering[0];
    ordering[0] = tmpI;
    V3D axis = t2;
    //now, determine which point to add to the list next...
    for (uint i = 1; i < points.size(); i++) {
        double minAngle = DBL_MAX;
        double maxLength = -DBL_MAX;
        int index = 0;
        for (uint j = 0; j < points.size(); j++) {
            if (V3D(points[i - 1], points[j]).norm() < 0.0001)
                continue;
            double angle = angleWith(V3D(points[i - 1], points[j]), axis, n);
            //we really don't expect to have negative angles here... if we do, it's just numerical errors...
            if (angle < -PI / 2)
                angle += 2 * PI;
            double len = V3D(points[i - 1], points[j]).norm();
            if (minAngle - angle >= 0.000001 || (fabs(minAngle - angle) < 0.000001 && len > maxLength)) {
                minAngle = angle;
                index = j;
                maxLength = len;
            }
        }

        //check to see if we're back at the start...
        if (V3D(points[0], points[index]).norm() < 0.000001) {
            //if so, we're done... remove everything else from ordering now...
            ordering.resize(i);
            i = (int)points.size();
        } else {
            P3D p = points[index];
            points[index] = points[i];
            points[i] = p;
            int tmpI = ordering[index];
            ordering[index] = ordering[i];
            ordering[i] = tmpI;
            axis = V3D(points[i - 1], points[i]);
        }
    }

    return ordering;
}

};  // namespace crl::loco
