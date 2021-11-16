
#include "../rays/env_light.h"

#include <limits>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    return image_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().

    return image_sampler.pdf(dir);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.

    
    float theta = std::acos(dir.y);
    float sinphi = dir.z/(std::sin(theta));
    float cosphi = dir.x/(std::cos(theta));
    float phi = std::atan2(sinphi, cosphi);
    float height = image.h * phi / PI_F;
    float width = image.w * theta / 2.0f / PI_F;
    size_t hsmall = std::floor(height);
    size_t wsmall = std::floor(width);
    float hdelta = (float)height - (float)hsmall;
    float wdelta = (float)width - (float)wsmall
    

    return Spectrum{};
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
