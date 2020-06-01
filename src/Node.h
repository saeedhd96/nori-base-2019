//
// Created by Saeed HD on 2/29/20.
//

#ifndef NORI_NODE_H
#define NORI_NODE_H

#include <vector>
#include "nori/mesh.h"


using namespace std;
NORI_NAMESPACE_BEGIN

class Node {
public:

//    Node(const vector<Node> &children, const vector<uint32_t> &triangles, int noTri);
    Node(const vector<uint32_t> &triangles, int noTri, BoundingBox3f bb, bool isLeaff);

    Node(BoundingBox3f bb, bool isLeaff);

    Node* getChild(int i);
    void setChild(int i, Node *node);

    const BoundingBox3f &getBbox() const;

    void setBbox(const BoundingBox3f &bbox);


    const vector<Node*> getCopyOfChildren() const;
    const vector<uint32_t> &getTriangles() const;

    int getNoTri() const;

    bool isLeaf1() const;

private:
    vector<Node*> children;
    vector<uint32_t> triangles;
    BoundingBox3f bbox;
    bool isLeaf;
    int noTri=0;
};

NORI_NAMESPACE_END

#endif //NORI_NODE_H
