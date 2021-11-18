#include "../util/rand.h"
#include "../lib/mathlib.h"
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include<queue>
#include <ctime>
#include <random>
#include <thread>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;21
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());
    
    new_node(box, 0, primitives.size(), 0, 0);
    root_idx = 0;
    if (primitives.size() == 0) return;
    // printf("Primitive Num %zu\n", primitives.size());
    std::queue<size_t>nodeQ;
    nodeQ.push(root_idx);
    while(!nodeQ.empty()){
        size_t num = nodeQ.front();
        nodeQ.pop();
        // printf("\nwe have %zu nodes left\n", nodeQ.size());
        Node* n = &nodes[num]; 
        if (num >= nodes.size()) {
            printf("node index out of range\n");
            continue;
        }
        // printf("Node %zu, left pos %f, %f, %f, right pos %f, %f, %f.\n", 
            // num, n.bbox.min[0], n.bbox.min[1], n.bbox.min[2], n.bbox.max[0], n.bbox.max[1], n.bbox.max[2]);
        if (nodes[num].size == 1) continue;
        // if (num == 0) printf("FIRST\n");

        float SN = nodes[num].bbox.surface_area();
        
        float bestC = FLT_MAX;
        int bestAxis = 4;
        
        int bestPartition = 0;
        int possibleNum = 16;
        for (int axis = 0; axis < 3; axis++){
            float min_point = nodes[num].bbox.min[axis];
            float max_point = nodes[num].bbox.max[axis];
            float range = max_point - min_point;
            // if (num == 0 && range <= 0.0) printf("Range %f\n", range);
            float gap = range / (float)possibleNum;
            if (range <= 0.0) continue;
            //  if (num == 0) printf("Second %d\n", axis);
            for (int potentialParti = 1; potentialParti < possibleNum; potentialParti++){
                float threahold = gap * (float)potentialParti + min_point;
                auto prim_start_iterator = primitives.begin() + nodes[num].start;
                auto prim_end_iterator = primitives.begin() + nodes[num].start + nodes[num].size;
                std::partition(prim_start_iterator, prim_end_iterator, 
                [threahold, axis](const Primitive& P)
                {
                    return P.bbox().center()[axis] < threahold;
                });
                
                size_t largestLeft = 0;
                BBox left, right;
                for(size_t i = nodes[num].start; i < nodes[num].start + nodes[num].size; i++){
                    if (primitives[i].bbox().center()[axis] < threahold)
                    {
                        largestLeft = i;
                        left.enclose(primitives[i].bbox());
                    }
                    else{
                        right.enclose(primitives[i].bbox());
                    }
                }
                float SA = left.surface_area();
                float SB = right.surface_area();
                float C = SA/SN * (largestLeft - nodes[num].start + 1) + SB/SN * (nodes[num].start + nodes[num].size - largestLeft) + 1;
                if (C < bestC){
                    bestAxis = axis;
                    bestC = C;
                    
                    bestPartition = potentialParti;
                }
            }
        }
        // printf("bestAxis %d\n", bestAxis);
        float min_point = nodes[num].bbox.min[bestAxis];
        float max_point = nodes[num].bbox.max[bestAxis];
        float range = max_point - min_point;
        float gap = range / (float)possibleNum;
        float threahold = gap * (float)bestPartition + min_point;
        auto prim_start_iterator = primitives.begin() + nodes[num].start;
        auto prim_end_iterator = primitives.begin() + nodes[num].start + nodes[num].size;
        std::partition(prim_start_iterator, prim_end_iterator, 
                [threahold, bestAxis](const Primitive& P)
                {
                    return P.bbox().center()[bestAxis] < threahold;
                });
                
               
        BBox left, right;
        size_t leftcnt=0, rightcnt=0;
        for(size_t i = nodes[num].start; i < nodes[num].start + nodes[num].size; i++){
            if (primitives[i].bbox().center()[bestAxis] < threahold)
            {
                left.enclose(primitives[i].bbox());
                leftcnt++;
            }
            else{
                right.enclose(primitives[i].bbox());
                rightcnt++;
            }
        }
        
        if (leftcnt == 0 || leftcnt == n->size)
        {
            BBox left1, right1;
            leftcnt = 0;
            rightcnt = 0;
            for(size_t i = nodes[num].start; i < nodes[num].start + nodes[num].size; i++){
                if (i < nodes[num].start + nodes[num].size/2)
                {
                    left1.enclose(primitives[i].bbox());
                    leftcnt++;
                }
                else{
                    right1.enclose(primitives[i].bbox());
                    rightcnt++;
                }
            }
            
            nodes[num].l = new_node(left1, nodes[num].start, nodes[num].size/2, 0, 0);
            nodes[num].r = new_node(right1, nodes[num].start + nodes[num].size/2, nodes[num].size - nodes[num].size/2, 0, 0);
            // if (num == 0)
            //     printf("New node %zu %zu\n", nodes[num].l, nodes[num].r);
            nodeQ.push(nodes[num].l);
            nodeQ.push(nodes[num].r);
        }
        else{
            nodes[num].l = new_node(left, nodes[num].start, leftcnt, 0, 0);
            nodes[num].r = new_node(right, nodes[num].start + leftcnt, rightcnt, 0, 0);
            // if (num == 0)
            //     printf("New nodes %zu %zu\n", nodes[num].l, nodes[num].r);
            nodeQ.push(nodes[num].l);
            nodeQ.push(nodes[num].r);
        }
        


    }

    // for (size_t i = 0; i < nodes.size(); i++)
    // {
    //     Node n = nodes[i];
    //     printf("Node %zu, min %f %f %f, max %f %f %f, left %zu, right %zu, size %zu\n", i, n.bbox.min[0],n.bbox.min[1], n.bbox.min[2], n.bbox.max[0], n.bbox.max[1], n.bbox.max[2], n.l, n.r, n.size);
    // }
    
    

}

template<typename Primitive> bool BVH<Primitive>::find_closest_hit(const Ray& ray, size_t nodeNum, Trace* closest, size_t level) const{
    if (primitives.size() == 0) return false;
    if (nodeNum >= nodes.size()){
        printf("hit out of bound\n"); return false;
    }
    Node node = nodes[nodeNum];
    std::uniform_real_distribution<float> d(0.0f, 1.0f);
    static thread_local std::mt19937 rng;
    if (node.is_leaf())
    {
        // if ( d(rng) < (0.000005f))
        // printf("%zu\n", level);
    //    if (node.size > 1) printf("multiple\n");
        Trace new_hit = primitives[node.start].hit(ray);
        if (new_hit.hit && new_hit.distance < closest->distance){
            *closest = new_hit;
            
            return true;
        }
        return false;
    }
    else{
        Vec2 hit1;
        Vec2 hit2;
        hit1 = ray.dist_bounds;
        hit2 = ray.dist_bounds;

        // printf("recurse\n");
        if (node.l == 0 || node.r == 0) printf("error!\n");
        bool h1 = nodes[node.l].bbox.hit(ray, hit1);
        bool h2 = nodes[node.r].bbox.hit(ray, hit2);
        
        if (!h1 && h2) return find_closest_hit(ray, node.r, closest, level + 1);
        if (h1 && !h2) return find_closest_hit(ray, node.l, closest, level + 1);
        if (!h1 && !h2) 
        {
            // if ( d(rng) < (0.000005f))
            // printf("early %zu\n", level);
            return false;
        }
        size_t first, second;
        Vec2 hitsecond;
        if (hit1.x <= hit2.x)
        {
            first = node.l;
            second = node.r;
            hitsecond = hit2;
        }
        else{
            first = node.r;
            second = node.l;
            hitsecond = hit1;
        }
        find_closest_hit(ray, first, closest, level + 1);
        if ((closest->hit && (hitsecond.x < closest->distance)) || !closest->hit)
        {
            find_closest_hit(ray, second, closest, level + 1);
            return true;
        }
        // if ( d(rng) < (0.000005f))
        //     printf("skip %zu\n", level);
        return true;
    }

}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    // Trace ret;
    // for(const Primitive& prim : primitives) {
    //     Trace hit = prim.hit(ray);
    //     ret = Trace::min(ret, hit);
    // }
    // return ret;

    Trace ret;
    // float dist = 0.0f;
    ret.distance = FLT_MAX;
    find_closest_hit(ray, 0, &ret, 0);
    
    return ret;
   

}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
