
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    // Vec3 inv = ray.dir.inverse();
    float tx0 = std::min((min.x - ray.point.x) / ray.dir.x, (max.x - ray.point.x) / ray.dir.x);
    float tx1 = std::max((min.x - ray.point.x) / ray.dir.x, (max.x - ray.point.x) / ray.dir.x);
    float ty0 = std::min((min.y - ray.point.y) / ray.dir.y, (max.y - ray.point.y) / ray.dir.y);
    float ty1 = std::max((min.y - ray.point.y) / ray.dir.y, (max.y - ray.point.y) / ray.dir.y);
    float tz0 = std::min((min.z - ray.point.z) / ray.dir.z, (max.z - ray.point.z) / ray.dir.z);
    float tz1 = std::max((min.z - ray.point.z) / ray.dir.z, (max.z - ray.point.z) / ray.dir.z);

    float x0 = std::max(tx0, times.x);
    float x1 = std::max(ty0, times.x);
    float x2 = std::max(tz0, times.x);
    float y0 = std::min(tx1, times.y);
    float y1 = std::min(ty1, times.y);
    float y2 = std::min(tz1, times.y);
    if (x0 > y0 || x1 > y1 || x2 > y2) return false;
    times.x = std::max({x0, x1, x2});
    times.y = std::min({y0, y1, y2});
    // printf("hit\n");
    return true;
//     for (int d = 0; d < 3; d++) {
//       float tt0 = std::min((min[d] - ray.point[d]) / ray.dir[d], (max[d] - ray.point[d]) / ray.dir[d]);
//       float tt1 = std::max((min[d] - ray.point[d]) / ray.dir[d], (max[d] - ray.point[d]) / ray.dir[d]);
//       times.x = std::max(times.x, tt0);
//       times.y = std::min(times.y, tt1);
//       if (times.x > times.y) return false;
//   }
//     printf("hit\n");
//   return true;
}
