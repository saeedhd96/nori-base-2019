//
// Created by Saeed HD on 3/1/20.
//


#include "Octree.h"

NORI_NAMESPACE_BEGIN


vector<BoundingBox3f> getOctets(const BoundingBox3f &bbox)
{
//    vector<BoundingBox3f> boxes(8);
    vector<BoundingBox3f> boxes2(8);
//    Point3f center((bbox.min[0]+bbox.max[0])/2.0,(bbox.min[1]+bbox.max[1])/2.0,(bbox.min[2]+bbox.max[2])/2.0);
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                int a = k*1+j*2+i*4*i;
//                Point3f newCorner(k==0?bbox.min[0]:bbox.max[0],j==0?bbox.min[1]:bbox.max[1], i==0?bbox.min[2]:bbox.max[2]);
//                TBoundingBox<Point3f> newBBox(center,newCorner);
//                Point3f newcenter = bbox.getCenter();
//                if(center==newcenter)
//                    cout<<"haha";
//                Point3f center = bbox.getCenter();
                TBoundingBox<Point3f> newBBox2(bbox.getCenter());
                newBBox2.expandBy(bbox.getCorner(a));

//                boxes[a] = newBBox;
                boxes2[a] = newBBox2;
            }
        }
    }

    return boxes2;
}

vector<BoundingBox3f> *triBox;

Node *buildTree(const BoundingBox3f &bbox , vector<uint32_t> triangles, int numberOfTriangles,  vector<BoundingBox3f> *tB)
{
    triBox = tB;

    return builder(bbox,triangles,numberOfTriangles, 0);
}


Node *builder(const BoundingBox3f &bbox , vector<uint32_t> triangles, int numberOfTriangles, int depth) {
    if (numberOfTriangles==0)
        return nullptr;

    if ( depth>7 || numberOfTriangles<=10) {
        Node *leaf = new Node(triangles,numberOfTriangles,bbox, true);

        numberOfTris+=numberOfTriangles;
        numberOfLeaves++;
        maxdepth = depth>maxdepth? depth:maxdepth;
        return leaf;
    }

    vector<vector<uint32_t>> tris(8, vector<uint32_t> (0));
    vector<int> numberOfTris(8,0);
    vector<BoundingBox3f> boxes = getOctets(bbox);

    for (int j = 0; j < numberOfTriangles; ++j) {
        bool a = false;
        for (int i = 0; i < 8; ++i) {
            if (boxes[i].overlaps((*triBox)[triangles[j]])){

                tris[i].push_back(triangles[j]) ;
                numberOfTris[i]+=1;
                a= true;
            }

        }
            if (!a && bbox.overlaps((*triBox)[triangles[j]]))
                cout<<"error";
    }

    Node *node;
    numberofInteriorNodes++;

    node = new Node(bbox,false);
    for (int i = 0; i < 8; ++i) {
//        cout<<"depth: "<<depth<< " child: "<< i<< endl;
        node->setChild(i,builder(boxes[i], tris[i],numberOfTris[i],depth+1));
    }
    return node;
}

void printStats()
{
    cout<<"Stats: "<<endl;
    cout<<" # of Triangles per leaf: "<<((float)numberOfTris)/numberOfLeaves<<endl;
    cout<< "Size of a Node: " << "80 bytes" <<endl;
    cout<< "# of leaves: " << numberOfLeaves <<endl;
    cout<< "# of interior node: " << numberofInteriorNodes <<endl;
    cout<< "max depth: " << maxdepth <<endl;


}

NORI_NAMESPACE_END
