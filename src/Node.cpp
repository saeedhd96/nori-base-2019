//
// Created by Saeed HD on 2/29/20.
//

#include "Node.h"



using namespace std;
NORI_NAMESPACE_BEGIN


Node::Node(BoundingBox3f bb, bool isLeaff):children(8, nullptr), bbox(bb), isLeaf(isLeaff){}

Node::Node(const vector<uint32_t> &triangles, int noTri,BoundingBox3f bb, bool isLeaff) : children(0),
                                                                           triangles(triangles),
                                                                           noTri(noTri), bbox(bb), isLeaf(isLeaff) {}

    const BoundingBox3f &Node::getBbox() const {
        return bbox;
    }

    void Node::setChild(int i, Node *node){
    Node::children[i] = node;
}


    const vector<Node*> Node::getCopyOfChildren() const {
        return children;
    }
    const vector<uint32_t> &Node::getTriangles() const {
        return triangles;
    }

    bool Node::isLeaf1() const {
        return isLeaf;
    }

    int Node::getNoTri() const {
        return noTri;
    }

    Node* Node::getChild(int i){
    return children[i];
}






NORI_NAMESPACE_END
