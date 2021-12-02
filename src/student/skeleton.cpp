
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    Vec3 close;
    Vec3 end2 = end - start;
    Vec3 pt2 = point - start;
    Vec3 closest;
    if (dot(end2, pt2) < 0)
        close = Vec3(0, 0, 0);
    else if (dot(end2, pt2) >= end2.norm())
        close = end2;
      else
        close = dot(end2, pt2) * end2;
    return start + close;
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos

    Mat4 Transformation = Mat4::I;
    for (Joint* joi = parent; joi != nullptr; joi = joi->parent){
        Transformation = Mat4::translate(joi->extent) * Transformation;
    }

    
    return Transformation;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton spaceh, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This sould
    // not take into account Skeleton::base_pos

    Mat4 Transformation = (Mat4::euler(pose));
    for (Joint* joi = parent; joi != nullptr; joi = joi->parent){
        Transformation = Mat4::translate(joi->extent) * Transformation;
        Transformation = (Mat4::euler(joi->pose)) * Transformation;
    }

    
    return Transformation;
}


Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    
    return j->joint_to_bind() * (j->extent) + base_pos;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    
    return j->joint_to_posed() * (j->extent) + base_pos;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.
    

    
    return Mat4::translate(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    

    return Mat4::translate(base_pos) * j->joint_to_posed() ;
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        Vec3 jEnd = end_of(j);
        Vec3 jStart = jEnd - j->extent; 
        for (size_t i = 0; i < verts.size(); i++){
            Vec3 iPos = verts[i].pos;
            Vec3 iPos_segment = closest_on_line_segment(jStart, jEnd, iPos);
            float dist = (iPos_segment - jStart).norm();
            if (dist < j->radius){
                map[i].push_back(j);
            }
        }
        
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();

    for(size_t i = 0; i < verts.size(); i++) {

        // Skin vertex i. Note that its position is given in object bind space.
        // float distSum = 0.0f;
        // for (size_t ji = 0; ji < map[i].size(); ji++){
        //     Vec3 jStart = end_of(map[i][ji]);
        //     Vec3 jEnd = jStart + map[i][ji]->extent; 
        //     Vec3 iPos = verts[i].pos;
        //     float dist = (closest_on_line_segment(jStart, jEnd, iPos) - jStart).norm();
        //     distSum += 1.0f/dist;
        // }
        // for (size_t ji = 0; ji < map[i].size(); ji++){
        //     Vec3 jStart = end_of(map[i][ji]);
        //     Vec3 jEnd = jStart + map[i][ji]->extent; 
        //     Vec3 iPos = verts[i].pos;
        //     float dist = (closest_on_line_segment(jStart, jEnd, iPos) - jStart).norm();
        //     distSum += 1.0f/dist;
        // }
        // Vec3 newPos = Vec3(0.0f);
        // for (size_t ji = 0; ji < map[i].size(); ji++){
        //     Vec3 jStart = end_of(map[i][ji]);
        //     Vec3 jEnd = jStart + map[i][ji]->extent; 
        //     Vec3 iPos = verts[i].pos;
        //     float dist = (closest_on_line_segment(jStart, jEnd, iPos) - jStart).norm();
        //     float w = 1.0f/dist/distSum;
        //     Vec3 vPosJ = joint_to_bind(map[i][ji]) * jStart - jStart;
        //     newPos = w * vPosJ + newPos;
        // }
        // verts[i].norm = verts[i].pos.normalize();



    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.
    
    //current and target should in joint space
    
    Mat4 Trans = joint_to_posed();
    Vec3 p =  target -   current;
    Vec3 xAxis = (Trans * Vec4(1, 0 ,0 ,0)).xyz().normalize();
    Vec3 yAxis = (Trans * Vec4(0 ,1, 0 ,0)).xyz().normalize();
    Vec3 zAxis = (Trans * Vec4(0 ,0 ,1, 0)).xyz().normalize();
    Vec3 pForcross = target - Trans * current;
    
    
    angle_gradient.x += dot(cross(xAxis, pForcross), p);
    angle_gradient.y += dot(cross(yAxis, pForcross), p);
    angle_gradient.z += dot(cross(zAxis, pForcross), p);




    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    size_t iterations = 100;
    float basic_step = 0.01f;
    for (size_t i = 0; i < iterations; i++){
        
        for_joints([&](Joint* j){j->angle_gradient = Vec3(0.0f);});
        
        for (size_t j = 0; j < active_handles.size(); j++){
            
            // Vec3 gradiant = Vec3{0.0f, 0.0f, 0.0f};
            for (Joint* k = active_handles[j]->joint; k != nullptr; k = k->parent){
                k->compute_gradient( active_handles[j]->target, posed_end_of(active_handles[j]->joint) - base_pos);
                // gradiant += k->angle_gradient;
            }
            
        }
        for_joints([&](Joint* j){j->pose -= basic_step * j->angle_gradient;});

    }
    


}
