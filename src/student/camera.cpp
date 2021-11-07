
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    float x = screen_coord.x;
    float y = screen_coord.y;
    float screenH = 2.0f * (float)std::tan(vert_fov/180.0f * PI_F / 2.0f);
    float screenW = screenH * aspect_ratio;
    Vec3 direction = Vec3((x-0.5f) * screenW, (y-0.5f) * screenH, -1.0f);
    // printf ("x and y, %f %f \n", (x-0.5f) * screenW, (y-0.5f) * screenH);
    

    Ray r = Ray(Vec3(0), direction.normalize());
    r.transform(iview);
    return r;
}
