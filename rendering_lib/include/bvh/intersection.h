#ifndef __INTERSECTION_H__
#define __INTERSECTION_H__

#include <geometry.h>
#include <bvh/bounds.h>

bool TriangleABBOverlap(const Bounds3f& abb, const Trianglef& triangle);
bool NaiveTriangleABBOverlap(const Bounds3f& abb, const Trianglef& triangle);

#endif