/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>

#include <Eigen/Geometry>
#include "Octree.h"

NORI_NAMESPACE_BEGIN

    typedef pair<Node*,float> pairs;
    static bool improved = true;

    void Accel::addMesh(Mesh *mesh) {
        if (m_mesh)
            throw NoriException("Accel: only a single mesh is supported!");
        m_mesh = mesh;
        vector<uint32_t> triangles(m_mesh->getTriangleCount(),0);
        static vector<BoundingBox3f> *triBox = new vector<BoundingBox3f>(0);
        for (uint32_t i = 0; i < m_mesh->getTriangleCount(); ++i) {
            triangles[i]= i;
            BoundingBox3f b = m_mesh->getBoundingBox(i);
            (*triBox).push_back(b);
        }
        m_bbox = m_mesh->getBoundingBox();

        auto start = std::chrono::high_resolution_clock::now();
        Node *Octree = buildTree(m_bbox,triangles,m_mesh->getTriangleCount(), triBox);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        cout << "Construction Time : "
             << duration.count() << " seconds" << endl;

        roottt = Octree;
        printStats();

        cout<<m_mesh->isEmitter()<<" salam \n";

    }

    void Accel::build() {
        /* Nothing to do here for now */
    }



    bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
        bool foundIntersection = false;  // Was an intersection found so far?
        uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

        Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

        /* Brute force search through all triangles */
        searchTheTree(shadowRay, ray, its, foundIntersection, f, roottt);

        if (!shadowRay && foundIntersection) {
            intersectionFound(its, f);

        }

        return foundIntersection;
    }



//    struct Local {
//        Local() {  }
//        bool operator()(Node *n1, Node *n2) {
//            if(n1== nullptr) cout<<"error";
//            ;
//        }
//
//    };

    bool comp(pairs p1, pairs p2) {
        return p1.second<p2.second;
    }



    void Accel::searchTheTree(bool shadowRay, Ray3f &ray, Intersection &its, bool &foundIntersection, uint32_t &f, Node *root) const {
        if(root== nullptr) return;
        if(foundIntersection && shadowRay) return;
        if (root->getBbox().rayIntersect(ray)) {
            if (root->isLeaf1()) {
                for (int i = 0; i < root->getNoTri(); ++i) {
                    if (forEachTriangleIntersect(its, ray, root->getTriangles()[i], foundIntersection, f, shadowRay) &&
                        shadowRay)
                        return;
                }
            } else {

                vector<pairs> toTraverse(0, pairs(nullptr,0.0));
                for (int i = 0; i < 8; ++i) {
                    if(root->getChild(i)== nullptr) continue;
                    float c,f;
                    if(root->getChild(i)->getBbox().rayIntersect(ray,c,f))
                    {
                        toTraverse.push_back(make_pair(root->getChild(i),c));
                    };
                }

                if (improved) std::sort(toTraverse.begin(),toTraverse.end(),comp);

                for (int i = 0; i < toTraverse.size(); ++i) {
                    searchTheTree(shadowRay, ray, its, foundIntersection, f, toTraverse[i].first);
                    if(improved && foundIntersection) return;
                }
            }
        }
    }




    bool Accel::forEachTriangleIntersect(Intersection &its, Ray3f &ray, uint32_t idx, bool &foundIntersection, uint32_t &f, bool shadowRay) const {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            foundIntersection = true;

            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            return true;
        }
        return false;
    }

    void Accel::intersectionFound(Intersection &its, uint32_t f) const {/* At this point, we now know that there is an intersection,
       and we know the triangle index of the closest such intersection.

       The following computes a number of additional properties which
       characterize the intersection (normals, texture coordinates, etc..)
    */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                     bary.y() * UV.col(idx1) +
                     bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                    (bary.x() * N.col(idx0) +
                     bary.y() * N.col(idx1) +
                     bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

NORI_NAMESPACE_END

