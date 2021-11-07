
#include "../rays/shapes.h"
#include "debug.h"
// #include <cstdio>

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3(); // where was the intersection?
    ret.normal = Vec3();   // what was the surface normal at the intersection?
    float od = dot(ray.point, ray.dir);

    float delta = (od * od - ray.point.norm_squared() + radius * radius);
    // printf("delta: %f\n", delta); 
    if (delta < 0.0){
        return ret;
    }
    float t1 = -dot(ray.point, ray.dir) - std::sqrt(delta);
    float t2 = -dot(ray.point, ray.dir) + std::sqrt(delta);
    if (ray.dist_bounds.x <= t1 && ray.dist_bounds.y >= t1){
        ret.hit = true;       // was there an intersection?
        ret.distance = t1;   // at what distance did the intersection occur?
        ret.position = ray.point + t1 * ray.dir; // where was the intersection?
        ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
        // printf("1\n");
    }
    else if (ray.dist_bounds.x <= t2 && ray.dist_bounds.y >= t2){
        ret.hit = true;       // was there an intersection?
        ret.distance = t2;   // at what distance did the intersection occur?
        ret.position = ray.point + t2 * ray.dir; // where was the intersection?
        ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
        // printf("2\n");
    }
    // printf("ok\n");
    return ret;
}

    

} // namespace PT
