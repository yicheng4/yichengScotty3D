
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Scene_Particles::Particle::update(const PT::Object& scene, float dt, float radius) {

    // TODO(Animation): Task 4

    // Compute the trajectory of this particle for the next dt seconds.

    // (1) Build a ray representing the particle's path if it travelled at constant velocity.
    float timePassed = 0.0f;

    while(timePassed < dt){
        Ray r;
        r.point = pos;
        r.dir = velocity.unit();
        // r.depth = 1;
        r.dist_bounds = Vec2(0.0, (dt - timePassed) * velocity.norm() + radius);
        // (2) Intersect the ray with the scene and account for collisions. Be careful when placing
        // collision points using the particle radius. Move the particle to its next position.
        PT::Trace trace = scene.hit(r);
        if (trace.hit){
            
            Vec3 normal = trace.normal.unit();
            if (dot(trace.normal, velocity) > 0) {
                normal = -normal;
            }
            timePassed += std::max(trace.distance / velocity.norm(), 0.0f);
            velocity += acceleration * (std::max(trace.distance / velocity.norm(), 0.0f));
            pos = trace.position - (radius) * velocity.unit();
            velocity = velocity - 2 * dot(velocity, normal) * normal;
            
        }
        else{
            pos += velocity * (dt - timePassed);
            // (3) Account for acceleration due to gravity.
            velocity += acceleration * (dt - timePassed); 
            timePassed = dt;
            break;
        }
        
    }
    
    
    // (4) Repeat until the entire time step has been consumed.

    // (5) Decrease the particle's age and return whether it should die.
    

    age -= dt;
    if (age < 0.0f)
        return false;

    return true;
}
