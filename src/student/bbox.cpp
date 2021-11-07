
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    float tx0 = std::min((min.x - ray.point.x) * ray.dir.x, (max.x - ray.point.x) * ray.dir.x);
    float tx1 = std::max((min.x - ray.point.x) * ray.dir.x, (max.x - ray.point.x) * ray.dir.x);
    float ty0 = std::min((min.y - ray.point.y) * ray.dir.y, (max.y - ray.point.y) * ray.dir.y);
    float ty1 = std::max((min.y - ray.point.y) * ray.dir.x, (max.y - ray.point.y) * ray.dir.y);
    float tz0 = std::min((min.z - ray.point.z) * ray.dir.z, (max.z - ray.point.z) * ray.dir.z);
    float tz1 = std::max((min.z - ray.point.z) * ray.dir.z, (max.z - ray.point.z) * ray.dir.z);
    if (tx0 > tx1 || ty0 > ty1 || tz0 > tz1) return false;
    times.x = std::max({tx0, ty0, tz0, times.x});
    times.y = std::min({tx1, ty1, tz1, times.y});
    return true;
}
