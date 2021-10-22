
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    // // Do not delete boundries and the last vertexes.
    validate();
    if (v->on_boundary() || ((long)faces.size() - (long)v->degree() < 1))
        return std::nullopt;
    
    using halfEdgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    using facePtr = Halfedge_Mesh::FaceRef;

    std::vector<halfEdgePtr>nbr_halfedge;
    std::vector<vertexPtr>nbr_vertex;
    std::vector<facePtr>nbr_face;
    halfEdgePtr this_h = v->halfedge();    
    halfEdgePtr start = v->halfedge();
    do {
        halfEdgePtr h;
        halfEdgePtr tmp;
        vertexPtr tmp2;
        bool started = false;
        for (h = this_h->next(); h->next() != this_h; h = h->next())  {
            if (started){
                nbr_halfedge.insert(std::find(nbr_halfedge.begin(), nbr_halfedge.end(), tmp),h);
            nbr_vertex.insert(std::find(nbr_vertex.begin(), nbr_vertex.end(), tmp2),h->vertex()); 
            }

            else{
                nbr_halfedge.push_back(h);
                nbr_vertex.push_back(h->vertex()); 
            }
            
            tmp = h;
            tmp2 = h->vertex();
            started = true;
        }
        nbr_face.push_back(this_h->face());
        this_h = this_h->twin()->next();
    } while(this_h != start);     
    size_t nbr_edge_cnt = nbr_halfedge.size();
    // printf("Yes%ld\n", nbr_edge_cnt);
    for (size_t i = 0; i < nbr_edge_cnt; i++) {   
        if (i == 0)
        {
            // printf("%u's next is %u\n",nbr_halfedge[i]->id(), nbr_halfedge[nbr_edge_cnt-1]->id());
            nbr_halfedge[i]->next() = nbr_halfedge[nbr_edge_cnt-1];}
        else{
            // printf("%u's next is %u\n",nbr_halfedge[i]->id(), nbr_halfedge[i-1]->id());
            nbr_halfedge[i]->next() = nbr_halfedge[i-1];
        }
        nbr_vertex[i]->halfedge() = nbr_halfedge[i];
        nbr_halfedge[i]->face() = nbr_face[0];
    }
    for (size_t i = 1; i < nbr_face.size(); i++)
    {
        Halfedge_Mesh::erase(nbr_face[i]);
    }
    this_h = v->halfedge();
    do{

        
            // h->vertex()->halfedge() = h;   
        halfEdgePtr h = this_h->twin()->next();
        // printf("erased %u, %u\n", this_h->id(), this_h->twin()->id());
        Halfedge_Mesh::erase(this_h->edge());
        Halfedge_Mesh::erase(this_h->twin());
        Halfedge_Mesh::erase(this_h);
        this_h = h;
    }while(this_h != v->halfedge());
    // printf("Yes2\n");
    Halfedge_Mesh::erase(v);
    nbr_face[0]->halfedge() = nbr_halfedge[0];

    return nbr_face[0]; 
    

}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    if (e->on_boundary() )
        return std::nullopt;
    using halfedgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    using facePtr = Halfedge_Mesh::FaceRef;
    
    halfedgePtr h0 = e->halfedge();
    halfedgePtr h1 = e->halfedge()->twin();
    vertexPtr v0= h0->vertex();
    vertexPtr v1 = h1->vertex();
    facePtr f0 = h0->face();
    facePtr f1 = h1->face();
    std::vector<halfedgePtr> workingh;
    if (v0->degree() < 2 
    || v1->degree() < 2)
        return std::nullopt;
    for (halfedgePtr h = h1->next(); h != h1; h = h->next()) {
        workingh.insert(workingh.begin(),h);
    }
    size_t right = workingh.size();
    v0->halfedge() = workingh[(signed)workingh.size()-1];
    for (halfedgePtr h = h0->next(); h != h0; h = h->next()) {
        workingh.insert(workingh.begin()+right,h);
    }
    
    v1->halfedge() = workingh[(signed)workingh.size()-1];
    for (long i = 0; i < (signed)workingh.size(); i++) {
        if (i == 0){
            // printf("%u's next is %u\n",workingh[i]->id(), workingh[(signed)workingh.size()-1]->id());
            workingh[i]->next() = workingh[(signed)workingh.size()-1];
        }
        else{
        // printf("%u's next is %u\n",workingh[i]->id(), workingh[i-1]->id());
        workingh[i]->next() = workingh[i - 1];}
        workingh[i]->face() = f0;
    }
    f0->halfedge() = workingh[0];
    // f1->halfege() =
    Halfedge_Mesh::erase(h0);
    Halfedge_Mesh::erase(h1);
    Halfedge_Mesh::erase(e);
    Halfedge_Mesh::erase(f1);
    
    return f0;

    
    
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    if (e->on_boundary())
        return std::nullopt;
    
    using halfEdgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    // using facePtr = Halfedge_Mesh::FaceRef;

    halfEdgePtr h0 = e->halfedge();
    halfEdgePtr h1 = h0->twin();
    vertexPtr v0 = h0->vertex();
    vertexPtr v1 = h1->vertex();
    std::set<vertexPtr>vtxIn;
    std::vector<halfEdgePtr> left, right;
    for(halfEdgePtr h = h1->next(); h->twin() != h1; h = h->twin()->next())
    {
        
        vtxIn.insert(h->twin()->vertex());
        left.push_back(h);
    }
    size_t righthand = 0;
    for(halfEdgePtr h = h0->next(); h->twin() != h0; h = h->twin()->next())
    {
        vertexPtr t = h->twin()->vertex();
        if (vtxIn.find(t) == vtxIn.end())
        {
           righthand++;
        }
        
    }
    if (righthand == 0)
    {
        return std::nullopt;
    }
    for(halfEdgePtr h = h0->next(); h->twin() != h0; h = h->twin()->next())
    {
        vertexPtr t = h->twin()->vertex();
        if (vtxIn.find(t) != vtxIn.end())
        {
            if (std::nullopt == Halfedge_Mesh::erase_edge(h->edge()))
            {
                return std::nullopt;
            }
            else
                if (validate() != std::nullopt)
                {
                    printf("NOO\n");
                }
        }
        
    }
    for(halfEdgePtr h = h0->next(); h->twin() != h0; h = h->twin()->next())
    {
        right.push_back(h);
        
    }

    
    if (faces.size() <= 2)
        return std::nullopt;
    size_t leftSize = left.size();
    size_t rightSize = right.size();
    if (leftSize < 1 || rightSize < 1) 
    {
        return std::nullopt;
    }
    vertexPtr v = new_vertex();
    v->pos = v0->pos + v1->pos;
    v->pos /= 2;
    // std::vector<vertexPtr> vtx;
    
    
    
    
    printf("YEs2\n");
    // Halfedge_Mesh::erase_edge(e);
    for(size_t i = 0; i < left.size(); i++)
    {
        halfEdgePtr h = left[i];
        h->vertex() = v;
        v->halfedge() = h;
       
    }
    for(size_t i = 0; i < right.size(); i++)
    {
        halfEdgePtr h = right[i];
        h->vertex() = v;
        v->halfedge() = h;
       
    }

    right[right.size()-1]->twin()->next() = left[0];
    left[left.size()-1]->twin()->next() = right[0];
    h0->face()->halfedge() = h0->face()->halfedge()->next();
    h1->face()->halfedge() = h1->face()->halfedge()->next();

    Halfedge_Mesh::erase(v0);
    Halfedge_Mesh::erase(v1);
    Halfedge_Mesh::erase(h0);
    Halfedge_Mesh::erase(h1);
    Halfedge_Mesh::erase(e);
    if (validate() != std::nullopt)
    {
        printf("NOO\n");
    }
   
    return v;



    
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    if (e->on_boundary()) 
        return std::nullopt;
    using halfedgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    using facePtr = Halfedge_Mesh::FaceRef;
    // using edgePtr = Halfedge_Mesh::EdgeRef;
    
    halfedgePtr h0 = e->halfedge();
    halfedgePtr h1 = e->halfedge()->twin();
    vertexPtr v0= h0->vertex();
    vertexPtr v1 = h1->vertex();
    facePtr f0 = h0->face();
    facePtr f1 = h1->face();
    if (v1->degree() <= 2 || v0->degree() <= 2)
        return std::nullopt;

    std::vector<halfedgePtr>left, right;
    std::vector<vertexPtr>vtx;
    // std::vector<edgePtr>egs;
    halfedgePtr iter;
    right.push_back(h1);
    size_t cnt = 0;
    for (iter = h1->next(); iter != h1; iter = iter->next())
    {
        right.push_back(iter);
        vtx.push_back(iter->vertex());
        // egs.push_back(iter->edge());
        cnt++;
    }
    if (cnt < 2) return std::nullopt;
    cnt = 0;
    left.push_back(h0);
    // printf("YES1\n");
    for (iter = h0->next(); iter != h0; iter = iter->next())
    {
        left.push_back(iter);
        vtx.push_back(iter->vertex());
        // egs.push_back(iter->edge());
        cnt++;
        
    }
    if (cnt < 2) return std::nullopt;
    printf("%lu %lu %lu\n", left.size(), right.size(), vtx.size());
    h0->vertex() = vtx[1];
    h1->vertex() = vtx[right.size()];
    v0->halfedge() = right[1];
    v1->halfedge() = left[1];
    // printf("YES1\n");
    vtx[1]->halfedge() = h0;
    vtx[right.size()]->halfedge() = h1;
    // right[0]->face() = f0;
    // left[0]->face() = f1;
    right[1]->face() = f0;
    left[1]->face() = f1;
    // right[right.size()-1]->face
    // printf("YES1\n");
    h1->next() = right[2];
    h0->next() = left[2];
    // printf("YES1\n");
    right[1]->next() = h0;
    left[1]->next() = h1;
    right[right.size()-1]->next() = left[1];
    left[left.size()-1]->next() = right[1];
    // printf("hi\n");
    std::set<HalfedgeRef> permutation;
    
    f1->halfedge() = h1;
    f0->halfedge() = h0;
    
    
    validate();

    // Check valid halfedge permutation
    
    return e;
    
    
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    if (e->on_boundary()) 
        return std::nullopt;
    using halfedgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    using facePtr = Halfedge_Mesh::FaceRef;
    using edgePtr = Halfedge_Mesh::EdgeRef;
    
    halfedgePtr h0 = e->halfedge();
    halfedgePtr h1 = e->halfedge()->twin();
    
    vertexPtr v0= h0->vertex();
    vertexPtr v1 = h1->vertex();
    
    facePtr f3 = new_face();
    facePtr f2 = new_face();
    
    halfedgePtr h;
    std::vector<halfedgePtr>left, right;
    for (h = h0->next(); h != h0; h = h->next())
    {
        left.push_back(h);
    }
    for (h = h1->next(); h != h1; h = h->next())
    {
        right.push_back(h);
    }
    if (left.size() != 2 || right.size() != 2)
    {
        return std::nullopt;
    }
    vertexPtr vL= left[1]->vertex();
    vertexPtr vR = right[1]->vertex();
    Halfedge_Mesh::erase_edge(e);
    // validate();
    facePtr f0 = left[0]->face();
    facePtr f1 = new_face();
    vertexPtr v = new_vertex();
    std::vector<edgePtr> new_eg;
    v->pos = (v0->pos + v1->pos + vL->pos + vR->pos)/4;
    for(int i =0; i < 4; i++)
    {
        new_eg.push_back(new_edge());
    }
    std::vector<halfedgePtr>new_half;
    for(int i =0; i < 8; i++)
    {
        new_half.push_back(new_halfedge());
        
    }
    for(int i =0; i < 4; i++)
    {
        new_half[i*2]->twin() = new_half[i*2 + 1];
        new_half[i*2 + 1]->twin() = new_half[i*2];
        new_half[i*2+1]->vertex() = v;
        new_half[i*2]->edge() = new_eg[i];
        new_half[i*2+1]->edge() = new_eg[i];
        new_eg[i]->halfedge() = new_half[i*2];
    }
    v->halfedge() = new_half[1];

    new_half[1]->next() = left[0];
    left[0]->next() = new_half[2];
    new_half[2]->next() = new_half[1];
    new_half[1]->face() = f0;
    new_half[2]->face() = f0;
    left[0]->face() = f0;
    f0->halfedge() = new_half[2];

    new_half[3]->next() = left[1];
    left[1]->next() = new_half[4];
    new_half[4]->next() = new_half[3];
    new_half[3]->face() = f2;
    new_half[4]->face() = f2;
    f2->halfedge() = new_half[4];
    left[1]->face() = f2;

    new_half[5]->next() = right[0];
    right[0]->next() = new_half[6];
    new_half[6]->next() = new_half[5];
    new_half[5]->face() = f3;
    new_half[6]->face() = f3;
    right[0]->face() = f3;
    f3->halfedge() = new_half[6];

    new_half[7]->next() = right[1];
    right[1]->next() = new_half[0];
    new_half[0]->next() = new_half[7];
    new_half[7]->face() = f1;
    new_half[0]->face() = f1;
    right[1]->face() = f1;
    f1->halfedge() = new_half[0];

    new_half[0]->vertex() = v1;
    new_half[4]->vertex() = v0;
    new_half[2]->vertex() = left[1]->vertex();
    new_half[6]->vertex() = right[1]->vertex();

    for(int i =0; i < 4; i++)
    {
        printf("%u\n",new_eg[i]->halfedge()->twin()->edge()->id());
        
    }
    std::set<HalfedgeRef> permutation;

    for(HalfedgeRef h = halfedges_begin(); h != halfedges_end(); h++) {

        if(herased.find(h) != herased.end()) continue;

       
        if(ferased.find(h->face()) != ferased.end()) {
            printf("h %u, f%u\n", h->id(), h->face()->id());
            // return {{h, "A live halfedge's face was erased!"}};
        }

        // Check whether each halfedge's next points to a unique halfedge
        if(permutation.find(h->next()) == permutation.end()) {
            permutation.insert(h->next());
        } else {
            printf( "A halfedge is the next of multiple halfedges!\n");
        }
    }
    for(VertexRef v = vertices_begin(); v != vertices_end(); v++) {

        if(verased.find(v) != verased.end()) continue;

        HalfedgeRef h = v->halfedge();
        // if(herased.find(h) != herased.end()) {
        //     return {{v, "A vertex's halfedge is erased!"}};
        // }

        do {
            if(h->vertex() != v) {
                printf("h %u, v%u, hv %u\n", h->id(), v->id(), h->vertex()->id());
                printf( "A vertex's halfedge does not point to that vertex!n");
                // return {{h, "A vertex's halfedge does not point to that vertex!"}};
            }
            h = h->twin()->next();
        } while(h != v->halfedge());
    }

    for(EdgeRef e = edges_begin(); e != edges_end(); e++) {

        if(eerased.find(e) != eerased.end()) continue;

        HalfedgeRef h = e->halfedge();
        // if(herased.find(h) != herased.end()) {
        //     return {{e, "An edge's halfedge is erased!"}};
        // }

        do {
            printf("%u\n", e->id());
            if(h->edge() != e) {
                printf( "A vertex's halfedge does not point to that vertex!n\n");
            }
            h = h->twin();
        } while(h != e->halfedge());
    }

    validate();
    return v;
}
    

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these 
    positions, as well as the normal and tangent offset fields to assign positions to 
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    using halfedgePtr = Halfedge_Mesh::HalfedgeRef;
    using vertexPtr = Halfedge_Mesh::VertexRef;
    using facePtr = Halfedge_Mesh::FaceRef;
    // using edgePtr = Halfedge_Mesh::EdgeRef;
    
    // printf("YEs\n");

    std::vector<halfedgePtr>out;
    std::vector<halfedgePtr>outer;
    std::vector<halfedgePtr>inner;

    std::vector<vertexPtr>vtx;
    std::vector<vertexPtr>new_vtx;
    std::vector<Halfedge_Mesh::EdgeRef>inside_egs;
    std::vector<Halfedge_Mesh::EdgeRef>connect_egs;
    std::vector<halfedgePtr>out_eg;
    std::vector<halfedgePtr>in_eg;
    std::vector<facePtr>side_face;

    halfedgePtr h0 = f->halfedge();
    halfedgePtr iter;
    
    // facePtr inner_face = new_face();
    long cnt = 0;
    out.push_back(h0);
    vtx.push_back(h0->vertex());
    for (iter = h0->next(); iter != h0; iter = iter->next())
    {
        out.push_back(iter);
        vtx.push_back(iter->vertex());
        // new_vtx.push_back(new_vertex());

        // egs.push_back(iter->edge());
        cnt++;
    }
    
    if (cnt <= 2)
        return std::nullopt;
    cnt ++;
    
    new_vtx.push_back(new_vertex());

        outer.push_back(new_halfedge());
        inner.push_back(new_halfedge());

        inside_egs.push_back(new_edge());
        connect_egs.push_back(new_edge());

        in_eg.push_back(new_halfedge());
        out_eg.push_back(new_halfedge());

        side_face.push_back(new_face());
    for (iter = h0->next(); iter != h0; iter = iter->next())
    {
       
        new_vtx.push_back(new_vertex());

        outer.push_back(new_halfedge());
        inner.push_back(new_halfedge());

        inside_egs.push_back(new_edge());
        connect_egs.push_back(new_edge());

        in_eg.push_back(new_halfedge());
        out_eg.push_back(new_halfedge());

        side_face.push_back(new_face());
    }

    for (long i = 0; i < cnt; i++)
    {
        out[i]->next() = in_eg[(i+1)%cnt];
        out[i]->vertex() = vtx[i];
        // printf("out %u, in %u, outer %u, inner %u, in edge %u, out eg %u \n", vtx[i]->id(), new_vtx[i]->id(), out[i]->id(), outer[i]->id(), in_eg[i]->id(), out_eg[i]->id());
        out[i]->face() = side_face[i];
        in_eg[(i+1)%cnt]->next() = outer[i];
        in_eg[(i+1)%cnt]->vertex() = vtx[(i+1)%cnt];
        in_eg[(i+1)%cnt]->twin() = out_eg[(i+1)%cnt];
        in_eg[(i+1)%cnt]->face() = side_face[i];
        in_eg[(i+1)%cnt]->edge() = connect_egs[(i+1)%cnt];
        outer[i]->next() = out_eg[i];
        outer[i]->vertex() = new_vtx[(i+1)%cnt];
        outer[i]->twin() = inner[i];
        outer[i]->face() = side_face[i];
        outer[i]->edge() = inside_egs[i];
        out_eg[i]->next() = out[i];
        out_eg[i]->vertex() = new_vtx[i];
        out_eg[i]->twin() = in_eg[i];
        out_eg[i]->face() = side_face[i];
        out_eg[i]->edge() = connect_egs[i];
        inner[i]->next() = inner[(i+1)%cnt];
        inner[i]->vertex() = new_vtx[i];
        inner[i]->twin() = outer[i];
        inner[i]->face() = f;
        inner[i]->edge() = inside_egs[i];

        vtx[i]->halfedge() = out[i];
        new_vtx[i]->halfedge() = inner[i];
        side_face[i]->halfedge() = out[i];
        inside_egs[i]->halfedge() = outer[i];
        connect_egs[i]->halfedge() = out_eg[i];

    }
    f->halfedge() = inner[0];

    

    validate();
    printf("YEs\n");
    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {
                                            
    if(flip_orientation) normal_offset = -normal_offset;
    printf("OHH\n");
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    

    // Vec3 normal = new_halfedges[0]->next()

    long cnt = (long)start_positions.size();
    if (cnt < 3) return;
    Vec3 center= start_positions[0];
    for(long i = 1; i < (long)start_positions.size(); i++)
    {
        center = center + start_positions[i];
        printf("%f, %f, %f\n", start_positions[i].x, start_positions[i].y, start_positions[i].z);
    }

    center = center / cnt;
    printf("center: %f, %f, %f\n\n", center.x, center.y, center.z);

    std::vector<Vec3>middlePos;
    for(long i = 0; i < (long)start_positions.size(); i++)
    {
        Vec3 pos = face->normal()*normal_offset + (tangent_offset * start_positions[i] + center) / (tangent_offset+1);
        middlePos.push_back(pos);
    }
    for (long i = 0; i < (long)new_halfedges.size(); i++) {
        printf("%f, %f, %f\n", middlePos[i].x, middlePos[i].y, middlePos[i].z);
        new_halfedges[i]->vertex()->pos = middlePos[i];
    }

    
    // bevel_face(face);


    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    
    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    
    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting 
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new. 
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    
    // Now flip any new edge that connects an old and new vertex.
    
    // Finally, copy new vertex positions into the Vertex::pos.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
