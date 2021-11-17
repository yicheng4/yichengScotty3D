
#include "../rays/env_light.h"
#include "../util/rand.h"
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
    // float sinphi = dir.z/(std::sin(theta));
    // float cosphi = dir.x/(std::cos(theta));
    float phi = std::atan2(dir.z, dir.x);
    if (phi < 0.0f) phi += 2.0f * PI_F;
    const auto [_w, _h] = image.dimension();
    size_t w = (size_t) _w;
    size_t h = (size_t) _h;
    float height = h * (1.0f - theta / PI_F);
    float width = w * phi / 2.0f / PI_F;
    if (std::floor(height) < 0.0f || std::floor(width) < 0.0f){
        return Spectrum{};
    }
    size_t hsmall = std::floor(height);
    size_t wsmall = std::floor(width);
    if ((hsmall + 1)>= h || (wsmall + 1)>= w)
        return Spectrum{};
    float hdelta = (float)height - (float)hsmall;
    float wdelta = (float)width - (float)wsmall;
    // printf("%zu %zu %zu %zu\n", wsmall, hsmall, w, h);

    Spectrum ret = (1.0f-wdelta) * ((1.0f-hdelta)*image.at(wsmall, hsmall) + hdelta*image.at(wsmall, hsmall + 1)) +
        (wdelta) * ((1.0f-hdelta)*image.at(wsmall+1, hsmall) + hdelta*image.at(wsmall+1, hsmall + 1));
    // if (RNG::coin_flip(0.00005f))
    // {
    //     Spectrum p = image.at(wsmall, hsmall);
    //     printf("%f %f %f\n", p.r, p.g, p.b);
    //     printf("%f %f %f\n", ret.r, ret.g, ret.b);
    //     printf("%f %f\n", wdelta, hdelta);
    //     printf("%zu %zu\n", wsmall, hsmall);
    //     printf("%zu %zu\n\n", w, h);
    // }    
    
    return ret;

    
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
