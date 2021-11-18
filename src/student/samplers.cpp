
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()


    return Vec2(RNG::unit(), RNG::unit()) * size;
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys;
    if (RNG::coin_flip(0.5f))
        ys = std::cos(theta);
    else
    {
        ys = -std::cos(theta);
    }
    
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
    // return Vec3{};

}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    total = 0.0f;
    for(size_t i = 0; i < (size_t) (w * h); i++)
    {
        size_t wi =(i % w);
        size_t hi = （size_t) std::floor((float)i / (float)w);
        if (i == 0) hi = 0;
        if (h == hi) hi -= 1;
    
        Spectrum pix = image.at(wi, (size_t)h - hi - 1);
        float pdfNew = pix.luma() * std::sin(hi * PI_F / h);  
        _pdf.push_back(pdfNew);
        if (_cdf.size() == 0){
            _cdf.push_back(pdfNew);
        }
        else 
            _cdf.push_back(_cdf[_cdf.size() - 1] + pdfNew);
        total += pdfNew;
    } 
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound
    float cdfPos = RNG::unit() * total;
    auto start_iterator = _cdf.begin();
    auto end_iterator = _cdf.end();
    auto find_iterator = std::upper_bound(start_iterator, end_iterator, cdfPos);
    size_t index = (size_t) (find_iterator - start_iterator);
    if (index >= _cdf.size())
        return Vec3{};
    size_t wi =(index % w);
    size_t hi = (size_t)std::floor((float)index / (float)w);
    float theta = wi * (2.0f * PI_F) / w;
    float phi = hi * PI_F / h;
    float xs = std::sin(phi) * std::cos(theta);
    float ys = std::cos(phi);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);

    
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?


    float phi = std::acos(dir.y);

    float theta = std::atan2(dir.z, dir.x);
    if (theta < 0.0f) theta += 2.0*PI_F;

    float hei = phi / PI_F * h;
    float wid = theta / 2.0f / PI_F * (float)w;
    if (wid < 0.0f) wid +=  (float)w;


    return _pdf[std::round(wid+ (float)w*hei)] / total * w * h / (2 * PI_F * PI_F * std::sin(phi));
    
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
