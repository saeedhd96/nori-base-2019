//
// Created by Saeed HD on 3/2/20.
//

#ifndef NORI_OCTREE_H
#define NORI_OCTREE_H


#include "Node.h"
#include "nori/mesh.h"
static int numberOfLeaves;
static int numberOfTris;
static int numberofInteriorNodes;
static int maxdepth;



NORI_NAMESPACE_BEGIN

    Node *buildTree(const BoundingBox3f &bbox , vector<uint32_t> triangles, int numberOfTriangles,  vector<BoundingBox3f> *tB);
    void printStats();


    vector<BoundingBox3f> getOctets(const BoundingBox3f &bbox);
Node *builder(const BoundingBox3f &bbox , vector<uint32_t> triangles, int numberOfTriangles, int depth);

NORI_NAMESPACE_END


#endif //NORI_OCTREE_H
